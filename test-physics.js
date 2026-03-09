/**
 * JREK Physics Test Harness
 *
 * Uses Puppeteer to launch the game in a headless browser,
 * simulate key inputs, and read back Matter.js physics state
 * to verify ragdoll behavior.
 *
 * Usage: node test-physics.js
 */

const puppeteer = require('puppeteer');
const http = require('http');
const fs = require('fs');
const path = require('path');

// --- Configuration ---
const GAME_DIR = __dirname;
const SERVER_PORT = 8377; // Unlikely to conflict
const VIEWPORT = { width: 1280, height: 720 };
const FRAME_MS = 1000 / 60; // ~16.7ms per physics frame

// --- Simple static file server ---
function killPort(port) {
    try {
        const { execSync } = require('child_process');
        execSync(`lsof -ti:${port} | xargs kill -9 2>/dev/null`, { stdio: 'ignore' });
    } catch (e) {
        // No process on port, that's fine
    }
}

function startServer() {
    return new Promise((resolve, reject) => {
        killPort(SERVER_PORT);
        const mimeTypes = {
            '.html': 'text/html',
            '.js': 'application/javascript',
            '.css': 'text/css',
            '.png': 'image/png',
            '.jpg': 'image/jpeg',
            '.svg': 'image/svg+xml',
        };

        const server = http.createServer((req, res) => {
            let filePath = path.join(GAME_DIR, req.url === '/' ? 'index.html' : req.url);
            const ext = path.extname(filePath);
            const contentType = mimeTypes[ext] || 'application/octet-stream';

            fs.readFile(filePath, (err, data) => {
                if (err) {
                    res.writeHead(404);
                    res.end('Not found');
                    return;
                }
                res.writeHead(200, { 'Content-Type': contentType });
                res.end(data);
            });
        });

        server.listen(SERVER_PORT, '127.0.0.1', () => {
            resolve(server);
        });
        server.on('error', reject);
    });
}

// --- Helper: read physics state from the page ---
async function getPhysicsState(page) {
    return page.evaluate(() => {
        const d = window._jrekDebug;
        if (!d || !d.runner) return null;
        const torso = d.runner.parts.torso;
        return {
            torsoX: torso.position.x,
            torsoY: torso.position.y,
            torsoAngle: torso.angle,
            torsoAngVel: torso.angularVelocity,
            torsoVelX: torso.velocity.x,
            torsoVelY: torso.velocity.y,
            fallen: d.runner.fallen,
            distance: d.runner.distance,
            rolling: d.runner.rolling,
            gameState: d.gameState(),
        };
    });
}

// --- Helper: wait N real milliseconds ---
function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

// --- Helper: sample physics state at intervals ---
async function sampleStates(page, durationMs, intervalMs = 100) {
    const samples = [];
    const startTime = Date.now();
    while (Date.now() - startTime < durationMs) {
        const state = await getPhysicsState(page);
        if (state) samples.push({ t: Date.now() - startTime, ...state });
        await sleep(intervalMs);
    }
    return samples;
}

// --- Helper: press/release keys via page.keyboard ---
async function pressKey(page, key) {
    await page.keyboard.down(key);
}

async function releaseKey(page, key) {
    await page.keyboard.up(key);
}

// --- Helper: start the game (press Space) and wait for it to initialize ---
async function startGame(page) {
    await page.keyboard.press('Space');
    // Wait for the game to initialize and physics to settle
    // The ragdoll needs a moment to reach equilibrium under gravity
    await sleep(1000);
    // Verify the game started
    const state = await getPhysicsState(page);
    if (!state) {
        throw new Error('Game did not start - _jrekDebug not available');
    }
    if (state.gameState !== 'playing') {
        throw new Error(`Game state is "${state.gameState}", expected "playing"`);
    }
    return state;
}

// --- Helper: reload and restart game fresh ---
async function resetGame(page) {
    await page.reload({ waitUntil: 'load', timeout: 20000 });
    // Wait for Matter.js to be available after reload
    await page.waitForFunction(() => typeof Matter !== 'undefined', { timeout: 10000 });
    await sleep(300);
}

// --- Helper: format angle in degrees ---
function deg(radians) {
    return (radians * 180 / Math.PI).toFixed(1);
}

// --- Test result tracking ---
const results = [];
function recordResult(name, passed, details) {
    results.push({ name, passed, details });
}

// ============================================================
// TEST 1: Idle Stability
// Do nothing for 3 seconds. Does the runner stay upright?
// ============================================================
async function testIdleStability(page) {
    console.log('\n--- Test 1: Idle Stability ---');
    console.log('Starting game and doing nothing for 3 seconds...');

    await resetGame(page);
    const initial = await startGame(page);
    console.log(`  Initial torso: x=${initial.torsoX.toFixed(1)} y=${initial.torsoY.toFixed(1)} angle=${deg(initial.torsoAngle)} deg`);

    const samples = await sampleStates(page, 3000, 100);

    // Take screenshot
    await page.screenshot({ path: '/tmp/jrek-test-idle.png', fullPage: false });

    const final = samples[samples.length - 1];
    const maxAngle = Math.max(...samples.map(s => Math.abs(s.torsoAngle)));
    const fell = samples.some(s => s.fallen);
    const yDrift = Math.abs(final.torsoY - initial.torsoY);

    console.log(`  Final torso:   x=${final.torsoX.toFixed(1)} y=${final.torsoY.toFixed(1)} angle=${deg(final.torsoAngle)} deg`);
    console.log(`  Max angle:     ${deg(maxAngle)} deg`);
    console.log(`  Y drift:       ${yDrift.toFixed(1)} px`);
    console.log(`  Fallen:        ${fell}`);

    // Pass criteria: stays upright (angle < 60 deg) and doesn't fall
    // Note: ragdoll bodies naturally wobble, so 60 deg is a reasonable threshold
    const passed = !fell && maxAngle < (Math.PI / 3);
    const status = passed ? 'PASS' : 'FAIL';
    const reason = fell
        ? 'Runner collapsed without input'
        : maxAngle >= (Math.PI / 3)
            ? `Torso tilted too far: ${deg(maxAngle)} deg (limit: 60 deg)`
            : 'Runner stayed upright';

    console.log(`  Result:        ${status} - ${reason}`);
    recordResult('Idle Stability', passed, {
        maxAngleDeg: parseFloat(deg(maxAngle)),
        yDrift: parseFloat(yDrift.toFixed(1)),
        fell,
        reason,
    });
}

// ============================================================
// TEST 2: Hold J
// Hold J key for 3 seconds. Measure movement and angle.
// ============================================================
async function testHoldJ(page) {
    console.log('\n--- Test 2: Hold J ---');
    console.log('Holding J key for 3 seconds...');

    await resetGame(page);
    const initial = await startGame(page);

    await pressKey(page, 'j');
    const samples = await sampleStates(page, 3000, 100);
    await releaseKey(page, 'j');

    await page.screenshot({ path: '/tmp/jrek-test-hold-j.png', fullPage: false });

    const final = samples[samples.length - 1];
    const maxAngle = Math.max(...samples.map(s => Math.abs(s.torsoAngle)));
    const totalXMovement = final.torsoX - initial.torsoX;
    const fell = samples.some(s => s.fallen);
    const maxVelX = Math.max(...samples.map(s => Math.abs(s.torsoVelX)));

    console.log(`  Initial pos:   x=${initial.torsoX.toFixed(1)} y=${initial.torsoY.toFixed(1)}`);
    console.log(`  Final pos:     x=${final.torsoX.toFixed(1)} y=${final.torsoY.toFixed(1)}`);
    console.log(`  X movement:    ${totalXMovement.toFixed(1)} px`);
    console.log(`  Max angle:     ${deg(maxAngle)} deg`);
    console.log(`  Max X vel:     ${maxVelX.toFixed(2)} px/frame`);
    console.log(`  Fallen:        ${fell}`);

    // Categorize what happened
    let behavior;
    if (fell) {
        behavior = 'fell';
    } else if (Math.abs(totalXMovement) > 200) {
        behavior = totalXMovement > 0 ? 'moved forward significantly' : 'moved backward significantly';
    } else if (Math.abs(totalXMovement) > 20) {
        behavior = totalXMovement > 0 ? 'shuffled forward' : 'shuffled backward';
    } else {
        behavior = 'stayed mostly in place';
    }

    // J should cause leg motion. It's OK to fall or move. We just want it to DO something.
    const didSomething = maxAngle > 0.05 || Math.abs(totalXMovement) > 5 || fell;
    const passed = didSomething;
    const status = passed ? 'PASS' : 'FAIL';

    console.log(`  Behavior:      ${behavior}`);
    console.log(`  Result:        ${status} - J key ${didSomething ? 'caused motion' : 'had no effect'}`);
    recordResult('Hold J', passed, {
        xMovement: parseFloat(totalXMovement.toFixed(1)),
        maxAngleDeg: parseFloat(deg(maxAngle)),
        fell,
        behavior,
    });
}

// ============================================================
// TEST 3: Hold J+E
// Hold both J and E for 3 seconds. Measure movement.
// ============================================================
async function testHoldJE(page) {
    console.log('\n--- Test 3: Hold J+E ---');
    console.log('Holding J+E keys for 3 seconds...');

    await resetGame(page);
    const initial = await startGame(page);

    await pressKey(page, 'j');
    await pressKey(page, 'e');
    const samples = await sampleStates(page, 3000, 100);
    await releaseKey(page, 'j');
    await releaseKey(page, 'e');

    await page.screenshot({ path: '/tmp/jrek-test-hold-je.png', fullPage: false });

    const final = samples[samples.length - 1];
    const maxAngle = Math.max(...samples.map(s => Math.abs(s.torsoAngle)));
    const totalXMovement = final.torsoX - initial.torsoX;
    const fell = samples.some(s => s.fallen);
    const maxVelX = Math.max(...samples.map(s => Math.abs(s.torsoVelX)));
    const maxVelY = Math.max(...samples.map(s => Math.abs(s.torsoVelY)));

    console.log(`  Initial pos:   x=${initial.torsoX.toFixed(1)} y=${initial.torsoY.toFixed(1)}`);
    console.log(`  Final pos:     x=${final.torsoX.toFixed(1)} y=${final.torsoY.toFixed(1)}`);
    console.log(`  X movement:    ${totalXMovement.toFixed(1)} px`);
    console.log(`  Max angle:     ${deg(maxAngle)} deg`);
    console.log(`  Max X vel:     ${maxVelX.toFixed(2)} px/frame`);
    console.log(`  Max Y vel:     ${maxVelY.toFixed(2)} px/frame`);
    console.log(`  Fallen:        ${fell}`);

    // Detect "flying" - torso going way above initial position
    const minY = Math.min(...samples.map(s => s.torsoY));
    const flew = (initial.torsoY - minY) > 100;

    let behavior;
    if (flew) {
        behavior = 'FLEW (torso launched upward)';
    } else if (fell) {
        behavior = 'fell';
    } else if (Math.abs(totalXMovement) > 200) {
        behavior = totalXMovement > 0 ? 'moved forward significantly' : 'moved backward';
    } else if (Math.abs(totalXMovement) > 20) {
        behavior = totalXMovement > 0 ? 'shuffled forward' : 'shuffled backward';
    } else {
        behavior = 'stayed mostly in place';
    }

    // J+E should cause more complex motion (hips + knees). Should cause motion.
    const didSomething = maxAngle > 0.05 || Math.abs(totalXMovement) > 5 || fell;
    // Flying is a physics bug
    const noFly = !flew;
    const passed = didSomething && noFly;
    const status = passed ? 'PASS' : 'FAIL';

    console.log(`  Flying:        ${flew}`);
    console.log(`  Behavior:      ${behavior}`);
    console.log(`  Result:        ${status} - ${flew ? 'Runner launched into the air (physics bug)' : didSomething ? 'J+E caused motion' : 'J+E had no effect'}`);
    recordResult('Hold J+E', passed, {
        xMovement: parseFloat(totalXMovement.toFixed(1)),
        maxAngleDeg: parseFloat(deg(maxAngle)),
        fell,
        flew,
        behavior,
    });
}

// ============================================================
// TEST 4: Alternate J/R
// Alternate pressing J and R every 0.5s for 5 seconds.
// This simulates a basic running gait.
// ============================================================
async function testAlternateJR(page) {
    console.log('\n--- Test 4: Alternate J/R ---');
    console.log('Alternating J and R every 0.5s for 5 seconds...');

    await resetGame(page);
    const initial = await startGame(page);

    const allSamples = [];
    const totalDuration = 5000;
    const toggleInterval = 500;
    const startTime = Date.now();
    let useJ = true;

    // Start first key
    await pressKey(page, 'j');

    while (Date.now() - startTime < totalDuration) {
        const elapsed = Date.now() - startTime;
        const shouldUseJ = Math.floor(elapsed / toggleInterval) % 2 === 0;

        if (shouldUseJ !== useJ) {
            if (shouldUseJ) {
                await releaseKey(page, 'r');
                await pressKey(page, 'j');
            } else {
                await releaseKey(page, 'j');
                await pressKey(page, 'r');
            }
            useJ = shouldUseJ;
        }

        const state = await getPhysicsState(page);
        if (state) allSamples.push({ t: elapsed, ...state });
        await sleep(50);
    }

    // Release all keys
    await releaseKey(page, 'j');
    await releaseKey(page, 'r');

    await page.screenshot({ path: '/tmp/jrek-test-alternate-jr.png', fullPage: false });

    const final = allSamples[allSamples.length - 1];
    const totalXMovement = final.torsoX - initial.torsoX;
    const maxAngle = Math.max(...allSamples.map(s => Math.abs(s.torsoAngle)));
    const fell = allSamples.some(s => s.fallen);
    const fellAt = fell ? allSamples.find(s => s.fallen)?.t : null;
    const stayedUpright = !fell;
    const totalDistance = final.distance;

    console.log(`  Initial pos:   x=${initial.torsoX.toFixed(1)}`);
    console.log(`  Final pos:     x=${final.torsoX.toFixed(1)}`);
    console.log(`  X movement:    ${totalXMovement.toFixed(1)} px (${totalXMovement > 0 ? 'forward' : 'backward'})`);
    console.log(`  Game distance: ${totalDistance.toFixed(1)} px`);
    console.log(`  Max angle:     ${deg(maxAngle)} deg`);
    console.log(`  Stayed upright: ${stayedUpright}${fell ? ` (fell at ${fellAt}ms)` : ''}`);

    // Pass: the runner should react to alternating inputs. Movement or falling both count.
    const passed = Math.abs(totalXMovement) > 5 || fell;
    const status = passed ? 'PASS' : 'FAIL';

    console.log(`  Result:        ${status} - Alternating J/R ${passed ? 'caused motion' : 'had no effect'}`);
    recordResult('Alternate J/R', passed, {
        xMovement: parseFloat(totalXMovement.toFixed(1)),
        gameDistance: parseFloat(totalDistance.toFixed(1)),
        maxAngleDeg: parseFloat(deg(maxAngle)),
        stayedUpright,
        fellAtMs: fellAt,
    });
}

// ============================================================
// TEST 5: Hold All Keys (Barrel Roll)
// Hold J+R+E+K for 3 seconds. Should trigger barrel roll.
// ============================================================
async function testBarrelRoll(page) {
    console.log('\n--- Test 5: Hold All Keys (Barrel Roll) ---');
    console.log('Holding J+R+E+K for 3 seconds...');

    await resetGame(page);
    const initial = await startGame(page);

    await pressKey(page, 'j');
    await pressKey(page, 'r');
    await pressKey(page, 'e');
    await pressKey(page, 'k');

    // Sample more frequently for better rotation tracking
    const samples = await sampleStates(page, 3000, 50);

    await releaseKey(page, 'j');
    await releaseKey(page, 'r');
    await releaseKey(page, 'e');
    await releaseKey(page, 'k');

    await page.screenshot({ path: '/tmp/jrek-test-barrel-roll.png', fullPage: false });

    const final = samples[samples.length - 1];
    const totalXMovement = final.torsoX - initial.torsoX;
    const totalRotation = final.torsoAngle - initial.torsoAngle;
    const maxAngVel = Math.max(...samples.map(s => Math.abs(s.torsoAngVel)));
    const isRolling = samples.some(s => s.rolling);
    const fell = samples.some(s => s.fallen);

    // Count full rotations by tracking angle deltas between samples
    // This handles wrapping better than cumulative angular velocity
    let cumulativeRotation = 0;
    for (let i = 1; i < samples.length; i++) {
        let delta = samples[i].torsoAngle - samples[i - 1].torsoAngle;
        // Normalize delta to [-PI, PI] to handle wrapping
        while (delta > Math.PI) delta -= 2 * Math.PI;
        while (delta < -Math.PI) delta += 2 * Math.PI;
        cumulativeRotation += delta;
    }
    const fullRotations = Math.abs(cumulativeRotation) / (2 * Math.PI);

    console.log(`  Rolling flag:       ${isRolling}`);
    console.log(`  X movement:         ${totalXMovement.toFixed(1)} px`);
    console.log(`  Cumulative rotation: ${(cumulativeRotation * 180 / Math.PI).toFixed(0)} deg (~${fullRotations.toFixed(1)} full rotations)`);
    console.log(`  Max angular vel:    ${maxAngVel.toFixed(3)} rad/frame`);
    console.log(`  Fallen:             ${fell}`);

    // Pass: barrel roll mode should activate (rolling flag), torso should spin
    // Quarter rotation (90 deg) is a reasonable threshold for 3 seconds
    const spun = Math.abs(cumulativeRotation) > (Math.PI / 2); // At least quarter rotation
    const passed = isRolling && spun && !fell;
    const status = passed ? 'PASS' : 'FAIL';

    let reason;
    if (!isRolling) reason = 'Rolling mode did not activate';
    else if (!spun) reason = 'Torso did not spin enough';
    else if (fell) reason = 'Runner fell during barrel roll';
    else reason = 'Barrel roll engaged successfully';

    console.log(`  Result:             ${status} - ${reason}`);
    recordResult('Barrel Roll', passed, {
        rollingActivated: isRolling,
        xMovement: parseFloat(totalXMovement.toFixed(1)),
        cumulativeRotationDeg: parseFloat((cumulativeRotation * 180 / Math.PI).toFixed(0)),
        fullRotations: parseFloat(fullRotations.toFixed(1)),
        fell,
        reason,
    });
}

// ============================================================
// MAIN
// ============================================================
async function main() {
    console.log('=== JREK Physics Test Harness ===');
    console.log(`Game directory: ${GAME_DIR}`);
    console.log(`Viewport: ${VIEWPORT.width}x${VIEWPORT.height}`);
    console.log('');

    // Start local server
    console.log(`Starting HTTP server on port ${SERVER_PORT}...`);
    const server = await startServer();
    _server = server;
    console.log('Server started.');

    // Launch browser
    console.log('Launching browser...');
    const browser = await puppeteer.launch({
        headless: 'shell',
        executablePath: '/usr/bin/google-chrome',
        args: [
            '--no-sandbox',
            '--disable-setuid-sandbox',
            '--disable-dev-shm-usage',
            '--disable-gpu',
            '--disable-extensions',
            '--window-size=1280,720',
        ],
    });

    _browser = browser;
    let page = await browser.newPage();
    await page.setViewport(VIEWPORT);

    // Navigate to game
    const url = `http://127.0.0.1:${SERVER_PORT}/`;
    console.log(`Loading game at ${url}...`);
    await page.goto(url, { waitUntil: 'load', timeout: 20000 });
    console.log('Page loaded.');

    // Wait for Matter.js to be available (loaded from CDN)
    await page.waitForFunction(() => typeof Matter !== 'undefined', { timeout: 10000 });
    console.log('Matter.js confirmed loaded.');

    // Run tests - each wrapped for crash resilience
    const tests = [
        ['Idle Stability', testIdleStability],
        ['Hold J', testHoldJ],
        ['Hold J+E', testHoldJE],
        ['Alternate J/R', testAlternateJR],
        ['Barrel Roll', testBarrelRoll],
    ];

    for (const [name, testFn] of tests) {
        try {
            await testFn(page);
        } catch (err) {
            console.error(`\n  ERROR in "${name}": ${err.message}`);
            recordResult(name, false, { error: err.message });

            // If browser crashed, try to recover
            if (err.message.includes('Target closed') || err.message.includes('Session closed') || err.message.includes('detached')) {
                console.log('  Browser connection lost. Relaunching...');
                try { await browser.close(); } catch (e) {}

                const newBrowser = await puppeteer.launch({
                    headless: 'shell',
                    executablePath: '/usr/bin/google-chrome',
                    args: [
                        '--no-sandbox',
                        '--disable-setuid-sandbox',
                        '--disable-dev-shm-usage',
                        '--disable-gpu',
                        '--disable-extensions',
                        '--window-size=1280,720',
                    ],
                });
                _browser = newBrowser;
                page = await newBrowser.newPage();
                await page.setViewport(VIEWPORT);
                await page.goto(`http://127.0.0.1:${SERVER_PORT}/`, { waitUntil: 'load', timeout: 20000 });
                await page.waitForFunction(() => typeof Matter !== 'undefined', { timeout: 10000 });
                console.log('  Browser relaunched. Continuing tests...');
            }
        }
    }

    // Summary
    console.log('\n\n========== SUMMARY ==========');
    console.log('');
    const passCount = results.filter(r => r.passed).length;
    const failCount = results.filter(r => !r.passed).length;

    for (const r of results) {
        const icon = r.passed ? 'PASS' : 'FAIL';
        console.log(`  [${icon}] ${r.name}`);
        if (r.details.reason) {
            console.log(`         ${r.details.reason}`);
        }
    }

    console.log('');
    console.log(`  ${passCount} passed, ${failCount} failed out of ${results.length} tests`);
    console.log('');
    console.log('Screenshots saved to /tmp/jrek-test-*.png');
    console.log('=================================');

    // Cleanup
    await browser.close();
    server.close();

    process.exit(failCount > 0 ? 1 : 0);
}

// Ensure cleanup on unexpected exit
let _server, _browser;
async function cleanup() {
    try { if (_browser) await _browser.close(); } catch (e) {}
    try { if (_server) _server.close(); } catch (e) {}
}
process.on('SIGINT', async () => { await cleanup(); process.exit(1); });
process.on('SIGTERM', async () => { await cleanup(); process.exit(1); });
process.on('uncaughtException', async (err) => {
    console.error('Uncaught exception:', err);
    await cleanup();
    process.exit(1);
});

main().catch(async (err) => {
    console.error('Fatal error:', err);
    await cleanup();
    process.exit(1);
});
