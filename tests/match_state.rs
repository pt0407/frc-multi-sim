//! Match state machine tests (issue #10, PLAN C3):
//! - a full match steps through disabled -> auto -> teleop -> endgame ->
//!   complete, with `match_state`/`match_time` on every SensorFrame
//! - actuator commands are ignored while disabled and after complete
//! - hub points are attributed to the phase they were scored in, and the
//!   placeholder climb bonus lands in the endgame bucket at completion
use frc_sim_m0::r#match::{MatchClock, MatchConfig, MatchPhase};
use frc_sim_m0::world::chassis;
use frc_sim_m0::world::field::{ClimbPointSpec, FieldSpec};
use frc_sim_m0::world::pieces::{HubZone, PieceTable, SpawnZone};
use frc_sim_m0::world::World;
use frc_sim_m0::{ActuatorFrame, ShotCommand};

const SEED: u64 = 0xbeef;

fn cmd() -> ActuatorFrame {
    ActuatorFrame {
        force: [0.0, 0.0],
        torque: 0.0,
        modules: None,
        intake_active: false,
        shoot: None,
    }
}

/// A ~2 s match: disabled 20t, auto 200t, teleop 400t, endgame 200t.
fn quick_match() -> MatchConfig {
    MatchConfig {
        disabled_s: 0.05,
        auto_s: 0.5,
        teleop_s: 1.0,
        endgame_s: 0.5,
        climb_bonus_points: 10,
        climb_radius_m: 1.0,
    }
}

#[test]
fn full_match_phase_transitions() {
    let cfg = quick_match();
    let clock = MatchClock::new(cfg);
    let mut w = World::with_config(
        &chassis::default_specs(),
        SEED,
        FieldSpec::default(),
        &PieceTable::default(),
    );
    w.match_clock = clock;

    let total = (2.05 * 400.0) as u64 + 8; // past the endgame boundary
    let mut seen: Vec<MatchPhase> = Vec::new();
    for tick in 0..total {
        w.step(&[None, None]);
        let f = w.sensor_frame(0, tick);
        assert_eq!(
            f.match_state,
            clock.phase_at(tick),
            "frame phase mismatch at tick {tick}"
        );
        assert_eq!(
            f.match_time,
            clock.time_remaining(tick),
            "frame clock mismatch at tick {tick}"
        );
        if seen.last() != Some(&f.match_state) {
            seen.push(f.match_state);
        }
    }
    assert_eq!(
        seen,
        vec![
            MatchPhase::Disabled,
            MatchPhase::Auto,
            MatchPhase::Teleop,
            MatchPhase::Endgame,
            MatchPhase::Complete,
        ],
        "phases did not progress in order"
    );
}

#[test]
fn commands_ignored_while_disabled_and_complete() {
    let cfg = quick_match();
    let mut w = World::with_config(
        &chassis::default_specs(),
        SEED,
        FieldSpec::default(),
        &PieceTable::default(),
    );
    w.match_clock = MatchClock::new(cfg);

    let mut drive = cmd();
    drive.force = [600.0, 0.0];

    // disabled window: 20 ticks at 0.05 s — robot must not move
    let x0 = w.sensor_frame(0, 0).position[0];
    for tick in 0..20u64 {
        w.step(&[Some(drive.clone()), None]);
        assert_eq!(w.sensor_frame(0, tick).match_state, MatchPhase::Disabled);
    }
    let x_after_disabled = w.sensor_frame(0, 20).position[0];
    assert!(
        (x_after_disabled - x0).abs() < 1e-3,
        "robot moved while disabled: {x0} -> {x_after_disabled}"
    );

    // once auto starts the same command applies and the robot drives
    for _ in 0..600 {
        w.step(&[Some(drive.clone()), None]);
    }
    let x_auto = w.sensor_frame(0, 220).position[0];
    assert!(
        x_auto - x_after_disabled > 0.2,
        "robot did not move once enabled (x={x_auto})"
    );

    // run the rest of the match; after complete, more commands do nothing
    for _ in 0..(2.05 * 400.0) as u64 {
        w.step(&[Some(drive.clone()), None]);
    }
    let x_end = w.sensor_frame(0, 0).position[0];
    for _ in 0..100 {
        w.step(&[Some(drive.clone()), None]);
    }
    let x_final = w.sensor_frame(0, 0).position[0];
    assert!(
        (x_final - x_end).abs() < 1e-3,
        "robot moved after match complete: {x_end} -> {x_final}"
    );
}

#[test]
fn scoring_is_attributed_to_phase_and_climb_bonus_lands() {
    // climb point on robot 0's spawn (x = -4); hub for alliance 0 to shoot at
    let mut field = FieldSpec::default();
    field.climb_points.push(ClimbPointSpec {
        name: "test-climb".to_string(),
        pos: [-4.0, 1.5, 0.0],
    });
    let table = PieceTable {
        zones: vec![SpawnZone {
            center: [4.0, 2.0],
            half_extents: [0.01, 0.01],
            count: 1,
        }],
        hubs: vec![HubZone {
            center: [-2.0, 0.0],
            radius: 0.8,
            entry_height: 1.0,
            alliance: 0,
        }],
    };
    // long auto so the lob shot is guaranteed to land while still auto;
    // teleop/endgame run briefly, then the match completes
    let cfg = MatchConfig {
        disabled_s: 0.0,
        auto_s: 20.0,
        teleop_s: 1.0,
        endgame_s: 0.5,
        climb_bonus_points: 10,
        climb_radius_m: 1.5,
    };
    let mut w = World::with_config(&chassis::default_specs(), SEED, field, &table);
    w.match_clock = MatchClock::new(cfg);

    // robot 0 lobs one held piece into its hub (same profile as the
    // game_pieces shot test — it scores within ~2 s = well inside auto)
    w.held[0] = 1;
    let mut shot = cmd();
    shot.shoot = Some(ShotCommand {
        launch_velocity: 4.0,
        hood_angle: 0.9,
        turret_yaw: 0.0,
        launch_height: 0.6,
    });
    w.step(&[Some(shot), None]);

    let mut scored_tick = None;
    for tick in 0..8000u64 {
        w.step(&[None, None]);
        if w.scores[0] > 0 && scored_tick.is_none() {
            scored_tick = Some(tick);
            break;
        }
    }
    let st = scored_tick.expect("shot never scored");
    assert_eq!(
        w.match_clock.phase_at(st),
        MatchPhase::Auto,
        "score landed outside auto (tick {st})"
    );
    assert_eq!(w.score_by_phase[0].auto, 1);
    assert_eq!(w.score_by_phase[0].teleop, 0);
    assert_eq!(w.score_by_phase[0].endgame, 0);

    // finish the match — robot 0 sits on the climb point, so the endgame
    // bucket picks up the bonus at the complete transition
    for tick in (st + 1)..8600u64 {
        w.step(&[None, None]);
        if w.match_clock.phase_at(tick) == MatchPhase::Complete {
            break;
        }
    }
    assert_eq!(w.match_clock.phase_at(w.tick()), MatchPhase::Complete);
    assert_eq!(w.scores[0], 11, "auto point + climb bonus expected");
    assert_eq!(w.score_by_phase[0].endgame, 10);

    // consistency: total == sum of phase buckets on both alliances
    for a in 0..2 {
        let p = &w.score_by_phase[a];
        assert_eq!(w.scores[a], p.auto + p.teleop + p.endgame);
    }
    // robot 1 (alliance 1, far from any climb point) got nothing
    assert_eq!(w.scores[1], 0);
}

#[test]
fn phase_override_holds_expires_and_scores() {
    // issue #33: manual phase override for rehearsals / viz demos
    let cfg = quick_match();
    let mut w = World::with_config(
        &chassis::default_specs(),
        SEED,
        FieldSpec::default(),
        &PieceTable::default(),
    );
    w.match_clock = MatchClock::new(cfg);

    // hold Endgame indefinitely (secs=0)
    w.set_phase(MatchPhase::Endgame, 0.0);
    assert_eq!(w.phase(), MatchPhase::Endgame);
    assert!(w.phase_override_active());
    let tr = w.time_remaining();
    assert!(tr > 1e6, "indefinite hold should report huge time_remaining, got {tr}");
    for _ in 0..50 {
        w.step(&[None, None]);
    }
    assert_eq!(w.phase(), MatchPhase::Endgame, "override must persist");

    // timed override: 0.2 s teleop = 80 ticks at 400 Hz
    w.set_phase(MatchPhase::Teleop, 0.2);
    for _ in 0..70 {
        w.step(&[None, None]);
    }
    assert_eq!(w.phase(), MatchPhase::Teleop);
    for _ in 0..20 {
        w.step(&[None, None]);
    }
    // override expired: clock resumes (we're still inside disabled_s=0.05? no —
    // 140+ ticks elapsed > 20-tick disabled, so auto)
    assert_eq!(w.phase(), MatchPhase::Auto);
    assert!(!w.phase_override_active());

    // manual score lands in the current phase bucket
    w.set_phase(MatchPhase::Endgame, 0.0);
    w.add_score(0, 25);
    assert_eq!(w.scores[0], 25);
    assert_eq!(w.score_by_phase[0].endgame, 25);
    w.clear_phase();
    assert!(!w.phase_override_active());
}
