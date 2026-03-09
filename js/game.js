/**
 * JREK - Main Game Loop
 * Ties together physics, rendering, controls, and UI
 */

// Polyfill for roundRect (older browsers)
if (!CanvasRenderingContext2D.prototype.roundRect) {
    CanvasRenderingContext2D.prototype.roundRect = function(x, y, w, h, r) {
        if (typeof r === 'number') r = [r, r, r, r];
        else if (!Array.isArray(r)) r = [0, 0, 0, 0];
        const [tl, tr, br, bl] = r;
        this.moveTo(x + tl, y);
        this.lineTo(x + w - tr, y);
        this.quadraticCurveTo(x + w, y, x + w, y + tr);
        this.lineTo(x + w, y + h - br);
        this.quadraticCurveTo(x + w, y + h, x + w - br, y + h);
        this.lineTo(x + bl, y + h);
        this.quadraticCurveTo(x, y + h, x, y + h - bl);
        this.lineTo(x, y + tl);
        this.quadraticCurveTo(x, y, x + tl, y);
        this.closePath();
        return this;
    };
}

(function() {
    const { Engine, Runner: MatterRunner, Events, Body } = Matter;

    // --- Canvas Setup ---
    const canvas = document.getElementById('gameCanvas');
    const ctx = canvas.getContext('2d');

    function resizeCanvas() {
        canvas.width = window.innerWidth;
        canvas.height = window.innerHeight;
    }
    resizeCanvas();
    window.addEventListener('resize', resizeCanvas);

    // --- Game State ---
    let engine = null;
    let runner = null;
    let terrain = null;
    let gameState = 'title'; // 'title', 'playing', 'fallen', 'finished'
    let startTime = 0;
    let elapsed = 0;
    let bestDistance = 0; // Best distance in miles
    let lastSpeed = 0;
    let speedSamples = [];
    let restartDelay = 0;

    // Key states
    const keys = {
        j: false,
        r: false,
        e: false,
        k: false,
    };

    // Camera
    let cameraX = 0;
    let cameraY = 0;
    let targetCameraX = 0;
    let targetCameraY = 0;

    // --- Detect mobile/touch device ---
    const isTouchDevice = ('ontouchstart' in window) || (navigator.maxTouchPoints > 0);
    const mobileControls = document.getElementById('mobileControls');

    function showMobileControls() {
        if (isTouchDevice && mobileControls) {
            mobileControls.classList.remove('hidden');
        }
    }

    function hideMobileControls() {
        if (mobileControls) {
            mobileControls.classList.add('hidden');
        }
    }

    // --- Input Handling (Keyboard) ---
    document.addEventListener('keydown', (ev) => {
        const key = ev.key.toLowerCase();

        if (key === ' ' || key === 'spacebar') {
            ev.preventDefault();
            if (gameState === 'title') {
                startGame();
            } else if (gameState === 'fallen' && Date.now() - restartDelay > 1000) {
                resetGame();
                startGame();
            }
            return;
        }

        if (gameState !== 'playing') return;

        if (key in keys) {
            keys[key] = true;
            ev.preventDefault();
        }
    });

    document.addEventListener('keyup', (ev) => {
        const key = ev.key.toLowerCase();
        if (key in keys) {
            keys[key] = false;
            ev.preventDefault();
        }
    });

    // --- Input Handling (Touch) ---
    // Tap title screen or fallen screen to start/restart
    document.getElementById('titleScreen').addEventListener('touchstart', (ev) => {
        ev.preventDefault();
        if (gameState === 'title') {
            startGame();
        }
    });

    canvas.addEventListener('touchstart', (ev) => {
        if (gameState === 'fallen' && Date.now() - restartDelay > 1000) {
            resetGame();
            startGame();
        }
    });

    // Touch buttons
    if (mobileControls) {
        const touchBtns = mobileControls.querySelectorAll('.touch-btn');
        touchBtns.forEach(btn => {
            const keyName = btn.getAttribute('data-key');

            btn.addEventListener('touchstart', (ev) => {
                ev.preventDefault();
                if (gameState === 'playing' && keyName in keys) {
                    keys[keyName] = true;
                    btn.classList.add('pressed');
                }
            });

            btn.addEventListener('touchend', (ev) => {
                ev.preventDefault();
                if (keyName in keys) {
                    keys[keyName] = false;
                    btn.classList.remove('pressed');
                }
            });

            btn.addEventListener('touchcancel', (ev) => {
                if (keyName in keys) {
                    keys[keyName] = false;
                    btn.classList.remove('pressed');
                }
            });

            // Also handle mouse for testing on desktop
            btn.addEventListener('mousedown', (ev) => {
                ev.preventDefault();
                if (gameState === 'playing' && keyName in keys) {
                    keys[keyName] = true;
                    btn.classList.add('pressed');
                }
            });

            btn.addEventListener('mouseup', (ev) => {
                if (keyName in keys) {
                    keys[keyName] = false;
                    btn.classList.remove('pressed');
                }
            });

            btn.addEventListener('mouseleave', (ev) => {
                if (keyName in keys) {
                    keys[keyName] = false;
                    btn.classList.remove('pressed');
                }
            });
        });
    }

    // Prevent zooming/scrolling on mobile when playing
    document.addEventListener('touchmove', (ev) => {
        if (gameState === 'playing') {
            ev.preventDefault();
        }
    }, { passive: false });

    // --- Game Lifecycle ---
    function startGame() {
        UI.hideTitleScreen();
        UI.hideFallMessage();
        UI.hideRestartPrompt();
        showMobileControls();

        // Create physics engine
        // Higher gravity = heavier, more grounded feel. QWOP uses ~2x Earth gravity.
        // This makes the inverted pendulum (torso) fall harder, requiring precise control.
        engine = Engine.create({
            gravity: { x: 0, y: 0.55 },
            constraintIterations: 6,   // Stiffer joints (default: 2)
            positionIterations: 10,    // Better position solving (default: 6)
        });

        // Create terrain
        terrain = Terrain.create();

        // Determine spawn position
        // Place runner so feet are just above the ground surface.
        // From torso center to feet bottom = TORSO_H/2 + UPPER_LEG_H + LOWER_LEG_H + FOOT_H
        // = 20 + 32 + 30 + 6 = 88px (from Runner body dimensions)
        const spawnX = 200;
        const groundY = Terrain.getGroundY(spawnX);
        const spawnY = groundY - 90; // torso center: feet will be ~2px above ground

        // Create runner
        runner = Runner.create(spawnX, spawnY);
        Runner.addToWorld(engine.world, runner);

        // Generate initial terrain (cover a good starting area)
        Terrain.generateTo(terrain, engine.world, Math.max(canvas.width + 500, 2000));

        // Reset camera
        cameraX = 0;
        cameraY = 0;
        targetCameraX = 0;
        targetCameraY = 0;

        // Reset state
        startTime = Date.now();
        elapsed = 0;
        speedSamples = [];
        lastSpeed = 0;
        gameState = 'playing';

        // Reset keys
        keys.j = false;
        keys.r = false;
        keys.e = false;
        keys.k = false;

        // Debug hook: expose internals for automated testing
        window._jrekDebug = { runner, engine, terrain, keys, gameState: () => gameState, Runner, Terrain };
    }

    function resetGame() {
        if (engine) {
            Engine.clear(engine);
        }
        runner = null;
        terrain = null;
        engine = null;
        gameState = 'title';
        hideMobileControls();
    }

    function handleFall() {
        gameState = 'fallen';
        restartDelay = Date.now();

        // Update best distance
        const miles = Terrain.pixelsToMiles(runner.maxDistance);
        if (miles > bestDistance) {
            bestDistance = miles;
        }

        UI.showFallMessage();

        // Show restart prompt after a brief delay
        setTimeout(() => {
            if (gameState === 'fallen') {
                UI.showRestartPrompt();
            }
        }, 1500);
    }

    function handleFinish() {
        gameState = 'finished';
        UI.showFinishScreen();
    }

    // Fixed physics timestep
    const PHYSICS_DT = 1000 / 60;
    let lastUpdateTime = 0;
    let accumulator = 0;

    // --- Physics Update ---
    function update() {
        if (gameState !== 'playing' && gameState !== 'fallen') return;

        // Update elapsed time
        if (gameState === 'playing') {
            elapsed = Date.now() - startTime;
        }

        // Apply controls
        if (gameState === 'playing') {
            Runner.applyControls(runner, keys);
        }

        // Step physics with fixed timestep
        Engine.update(engine, PHYSICS_DT);

        // Update distance
        const dist = Runner.updateDistance(runner);
        const miles = Terrain.pixelsToMiles(dist);

        // Calculate speed/pace
        if (gameState === 'playing') {
            const torso = runner.parts.torso;
            const speed = Math.abs(torso.velocity.x);
            speedSamples.push(speed);
            if (speedSamples.length > 30) speedSamples.shift();
            const avgSpeed = speedSamples.reduce((a, b) => a + b, 0) / speedSamples.length;
            // Convert pixels/frame to min/mile
            // speed is in pixels per frame (at 60fps)
            // pixels per second = speed * 60
            // miles per second = (speed * 60) / PIXELS_PER_MILE
            // seconds per mile = PIXELS_PER_MILE / (speed * 60)
            // minutes per mile = PIXELS_PER_MILE / (speed * 60 * 60)
            if (avgSpeed > 0.5) {
                lastSpeed = Terrain.PIXELS_PER_MILE / (avgSpeed * 60 * 60);
            } else {
                lastSpeed = 0;
            }
        }

        // Check for fall
        if (gameState === 'playing') {
            const groundY = Terrain.getGroundY(runner.parts.torso.position.x);
            if (Runner.checkFallen(runner, groundY)) {
                handleFall();
            }
        }

        // Check for finish
        if (gameState === 'playing' && Terrain.isFinished(dist)) {
            handleFinish();
        }

        // Generate more terrain ahead
        if (terrain) {
            const lookAhead = cameraX + canvas.width + 800;
            if (lookAhead > terrain.generated - 500) {
                Terrain.generateTo(terrain, engine.world, terrain.generated + 2000);
            }
        }

        // Update camera
        updateCamera();
    }

    function updateCamera() {
        if (!runner) return;

        const center = Runner.getCenter(runner);
        // Camera leads ahead of the runner
        targetCameraX = center.x - canvas.width * 0.35;
        targetCameraY = center.y - canvas.height * 0.55;

        // Clamp camera Y so we don't see above the sky too much
        targetCameraY = Math.max(-100, targetCameraY);
        targetCameraY = Math.min(Terrain.BASE_GROUND_Y - canvas.height * 0.3, targetCameraY);

        // Don't let camera go left of start
        targetCameraX = Math.max(-50, targetCameraX);

        // Smooth camera follow
        const smoothing = 0.08;
        cameraX += (targetCameraX - cameraX) * smoothing;
        cameraY += (targetCameraY - cameraY) * smoothing;
    }

    // --- Rendering ---
    function render() {
        ctx.clearRect(0, 0, canvas.width, canvas.height);

        if (gameState === 'title') {
            // Title screen is handled by CSS/HTML
            return;
        }

        if (!terrain || !runner) return;

        // Draw terrain (background, trail, obstacles)
        Terrain.render(ctx, terrain, cameraX, cameraY, canvas.width, canvas.height);

        // Draw runner
        Runner.render(ctx, runner, cameraX, cameraY);

        // Draw HUD
        const miles = Terrain.pixelsToMiles(runner.distance);
        const currentMarker = Terrain.getCurrentMarker(miles);
        const nextMarker = Terrain.getNextMarker(miles);

        UI.renderHUD(ctx, canvas.width, canvas.height, {
            distance: runner.distance,
            miles: miles,
            pace: lastSpeed,
            elapsed: elapsed,
            fallen: runner.fallen,
            currentMarker: currentMarker,
            nextMarker: nextMarker,
            keys: keys,
        });

        // Start message
        UI.renderStartMessage(ctx, canvas.width, canvas.height, elapsed);

        // Personal best
        UI.renderPersonalBest(ctx, canvas.width, bestDistance);
    }

    // --- Main Loop ---
    function gameLoop() {
        update();
        render();
        requestAnimationFrame(gameLoop);
    }

    // --- Initialize ---
    UI.init();
    gameLoop();

})();
