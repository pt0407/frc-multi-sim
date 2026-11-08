// Headless visual self-test for the sim viz.
// Requires: dev server (vite) + physics server running.
// Usage: node scripts/selftest.js [mode] [out.png] [key] [x,y,w,h]
//   mode: anim | practice | match  (auto-clicks the home button via ?param)
//   key:  optional keypress before screenshot (e.g. "2" for robot cam)
//   clip: optional "x,y,w,h" screenshot crop
// Exits nonzero on page errors / console errors so it can gate CI later.
const puppeteer = require('puppeteer-core');

const mode = process.argv[2] || 'practice';
const out = process.argv[3] || `/tmp/sim-${mode}.png`;
const key = process.argv[4];
const clip = process.argv[5];
const CHROME = process.env.CHROME_BIN
  || '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome';
const BASE = process.env.SIM_URL || 'http://localhost:5173';
const WAIT_MS = parseInt(process.env.SIM_WAIT_MS || '25000', 10);

(async () => {
  const browser = await puppeteer.launch({
    executablePath: CHROME,
    headless: 'new',
    args: ['--window-size=1600,900'],
    defaultViewport: { width: 1600, height: 900 },
  });
  const page = await browser.newPage();
  const errors = [];
  page.on('console', (m) => {
    const t = m.text();
    if (m.type() === 'error' && !t.includes('404')) errors.push(t.slice(0, 220));
    else console.log(`[${m.type()}]`, t.slice(0, 220));
  });
  page.on('pageerror', (e) => errors.push('PAGEERROR: ' + e.message));
  await page.goto(`${BASE}/?${mode}`, { waitUntil: 'load', timeout: 60000 });
  await new Promise((r) => setTimeout(r, WAIT_MS));
  if (key) {
    await page.keyboard.press(key);
    await new Promise((r) => setTimeout(r, 1500));
  }
  if (clip) {
    const [x, y, w, h] = clip.split(',').map(Number);
    await page.screenshot({ path: out, clip: { x, y, width: w, height: h } });
  } else {
    await page.screenshot({ path: out });
  }
  console.log('saved', out);
  console.log('TITLE:', await page.title());
  if (errors.length) {
    console.log('ERRORS:\n' + errors.join('\n'));
    await browser.close();
    process.exit(1);
  }
  await browser.close();
})();
