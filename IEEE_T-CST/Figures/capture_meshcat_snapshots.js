// Capture 3D Meshcat simulation snapshots using Playwright
// Opens the Drake Meshcat replay HTML, seeks to specific times,
// repositions camera to frame the formation, and captures PNGs
//
// NOTE: Meshcat/Drake uses Y-up coordinate convention in Three.js.
// The "payload" object's worldPos.y is the vertical (altitude) axis.

const { chromium } = require('playwright');
const path = require('path');
const fs = require('fs');

const OUTPUT_DIR = path.join(__dirname, 'scenario_snapshots');
fs.mkdirSync(OUTPUT_DIR, { recursive: true });

const SCENARIOS = [
  {
    name: 'three_drones',
    html: '/workspaces/Tether_Grace/outputs/full_drake_fault_batch/three_drones/full_drake_meshcat_replay.html',
    snapshots: [
      { time: 6.0,  label: 'a_pre_fault' },
      { time: 7.5,  label: 'b_after_snap' },
      { time: 12.0, label: 'c_settled' },
    ],
  },
  {
    name: 'five_drones',
    html: '/workspaces/Tether_Grace/outputs/full_drake_fault_batch/five_drones/full_drake_meshcat_replay.html',
    snapshots: [
      { time: 6.0,  label: 'a_pre_fault' },
      { time: 8.0,  label: 'b_after_snap' },
      { time: 15.0, label: 'c_settled' },
    ],
  },
  {
    name: 'seven_drones',
    html: '/workspaces/Tether_Grace/outputs/full_drake_fault_batch/seven_drones/full_drake_meshcat_replay.html',
    snapshots: [
      { time: 6.0,  label: 'a_pre_fault' },
      { time: 8.0,  label: 'b_after_snap' },
      { time: 18.0, label: 'c_settled' },
    ],
  },
];

(async () => {
  const browser = await chromium.launch({
    headless: true,
    args: ['--no-sandbox', '--disable-gpu', '--use-gl=swiftshader'],
  });

  for (const scenario of SCENARIOS) {
    console.log(`\n=== ${scenario.name} ===`);
    const context = await browser.newContext({
      viewport: { width: 1600, height: 1000 },
      deviceScaleFactor: 2,
    });
    const page = await context.newPage();

    const fileUrl = 'file://' + scenario.html;
    console.log(`  Loading: ${fileUrl}`);
    await page.goto(fileUrl, { waitUntil: 'networkidle', timeout: 60000 });
    await page.waitForTimeout(4000);

    const duration = await page.evaluate(() => viewer.animator.duration);
    console.log(`  Animation duration: ${duration}s`);

    for (const snap of scenario.snapshots) {
      console.log(`  Seeking to t=${snap.time}s...`);

      // Seek the animation
      await page.evaluate((t) => {
        viewer.animator.seek(t);
        if (viewer.animator.mixer) viewer.animator.mixer.update(0);
      }, snap.time);
      await page.waitForTimeout(1000);

      // Find the payload position in the scene to center the camera on it
      const payloadPos = await page.evaluate(() => {
        const Vector3 = viewer.camera.position.constructor;
        let pos = null;
        viewer.scene.traverse((child) => {
          if (child.name === 'payload') {
            const wp = new Vector3();
            child.getWorldPosition(wp);
            pos = { x: wp.x, y: wp.y, z: wp.z };
          }
        });
        return pos;
      });

      if (payloadPos) {
        console.log(`    Payload at: [${payloadPos.x.toFixed(2)}, ${payloadPos.y.toFixed(2)}, ${payloadPos.z.toFixed(2)}]`);

        // Position camera: close to the formation, slightly above and to the side
        // Y is UP in Meshcat/Three.js, so camera.y controls altitude
        await page.evaluate((pp) => {
          const camDist = 2.2;
          // Look at a point between payload and drones (midway in Y/altitude)
          const targetY = pp.y + 0.8;
          viewer.camera.position.set(
            pp.x + camDist * 0.9,   // offset in x
            targetY + camDist * 0.7, // above the midpoint
            pp.z - camDist * 0.8    // offset in z
          );
          viewer.controls.target.set(pp.x, targetY, pp.z);
          viewer.controls.update();
          viewer.needs_render = true;
        }, payloadPos);
      }

      await page.waitForTimeout(800);

      // Force render
      await page.evaluate(() => {
        viewer.needs_render = true;
        if (viewer.animate) viewer.animate();
      });
      await page.waitForTimeout(500);

      // Hide GUI
      await page.evaluate(() => {
        document.querySelectorAll('.dg').forEach(g => g.style.display = 'none');
      });
      await page.waitForTimeout(200);

      const outPath = path.join(OUTPUT_DIR, `${scenario.name}_${snap.label}.png`);
      await page.screenshot({ path: outPath, fullPage: false });
      console.log(`    Saved: ${outPath}`);

      // Restore GUI
      await page.evaluate(() => {
        document.querySelectorAll('.dg').forEach(g => g.style.display = '');
      });
    }

    await context.close();
  }

  await browser.close();
  console.log('\nAll snapshots captured.');
})();
