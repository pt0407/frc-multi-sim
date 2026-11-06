#!/usr/bin/env python3
"""Reference sim agent — the smallest useful robot-code adapter.

Usage: pip install websockets && python agents/example_client.py <robot_id>

Shows the full contract: read SensorFrame at 50Hz, emit ActuatorFrame.
A real team's robot code goes where `decide()` is — this stub just
drives a square patrol around the field.
"""
import asyncio, json, math, sys
import websockets

PATROL = [(-2.0, -2.0), (-2.0, 2.0), (2.0, 2.0), (2.0, -2.0)]
KP, MAX_F, K_D = 900.0, 1200.0, 200.0

def decide(s, wp):
    """Return ActuatorFrame for one sensor tick. Pure, deterministic."""
    dx = PATROL[wp][0] - s["position"][0]
    dz = PATROL[wp][1] - s["position"][2]
    dist = math.hypot(dx, dz)
    if dist < 0.3:
        return None, True  # waypoint reached
    # position error * P - velocity * D, clipped to drivetrain force
    fx = KP * dx - K_D * s["linvel"][0]
    fz = KP * dz - K_D * s["linvel"][2]
    f = math.hypot(fx, fz)
    if f > MAX_F:
        fx, fz = fx * MAX_F / f, fz * MAX_F / f
    return {
        "force": [fx, fz], "torque": 0.0,
        "intake_active": True,
    }, False

async def main():
    rid = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    async with websockets.connect(f"ws://127.0.0.1:9770/robot/{rid}") as ws:
        # handshake: the server rejects connections without a Hello
        # carrying the matching protocol_version
        await ws.send(json.dumps({
            "type": "Hello",
            "name": "example-python-agent",
            "robot_id": rid,
            "protocol_version": 1,
        }))
        wp = 0
        async for raw in ws:
            msg = json.loads(raw)
            if msg.get("type") == "Welcome":
                continue
            if msg.get("type") == "Error":
                print("rejected:", msg["message"]); return
            if msg.get("type") != "Sensor":
                continue
            s = msg  # fields are flattened onto the tagged envelope
            frame, done = decide(s, wp)
            if done:
                wp = (wp + 1) % len(PATROL)
            if frame:
                await ws.send(json.dumps({"type": "Actuator", **frame}))

if __name__ == "__main__":
    asyncio.run(main())
