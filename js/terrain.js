/**
 * JREK Terrain - Western States trail course generation
 * Procedural side-scrolling terrain with parallax backgrounds
 */

const Terrain = (function() {
    const { Bodies, Composite, Body } = Matter;

    // Terrain generation parameters
    const SEGMENT_WIDTH = 60;       // Width of each ground segment
    const GROUND_THICKNESS = 40;    // How thick the ground bodies are
    const BASE_GROUND_Y = 500;      // Base Y position of the ground surface
    const TERRAIN_AMPLITUDE = 30;   // Max height variation for gentle undulations
    const ROCK_CHANCE = 0.06;       // Probability of a rock obstacle per segment
    const ROOT_CHANCE = 0.04;       // Probability of a root obstacle per segment

    // Conversion: pixels to miles (the course is 100 miles)
    // Let's say 1 mile = 2000 pixels for reasonable gameplay
    const PIXELS_PER_MILE = 2000;

    // Mile markers on the Western States course
    const MILE_MARKERS = [
        { mile: 0, name: 'Squaw Valley Start', elevation: 6200 },
        { mile: 15.8, name: 'Robinson Flat', elevation: 6730 },
        { mile: 24.4, name: 'Duncan Canyon', elevation: 4760 },
        { mile: 30.3, name: 'Last Chance', elevation: 5230 },
        { mile: 43.3, name: "Devil's Thumb", elevation: 4760 },
        { mile: 47.8, name: 'El Dorado Creek', elevation: 2500 },
        { mile: 55.7, name: 'Michigan Bluff', elevation: 3590 },
        { mile: 62, name: 'Foresthill', elevation: 3265 },
        { mile: 70, name: "Cal Street", elevation: 1360 },
        { mile: 78, name: 'River Crossing', elevation: 740 },
        { mile: 80, name: "Green Gate", elevation: 1400 },
        { mile: 85.2, name: 'Auburn Lake Trails', elevation: 1580 },
        { mile: 90.7, name: 'Quarry Road', elevation: 1150 },
        { mile: 93.5, name: 'No Hands Bridge', elevation: 740 },
        { mile: 96.8, name: 'Robie Point', elevation: 1200 },
        { mile: 100, name: 'Placer High School Track - FINISH', elevation: 1210 },
    ];

    // Colors
    const TRAIL_COLOR = '#8B6914';
    const TRAIL_SURFACE = '#a07830';
    const DIRT_DARK = '#5c3d1a';
    const ROCK_COLOR = '#8a8a7a';
    const ROCK_DARK = '#6a6a5a';
    const ROOT_COLOR = '#4a3520';

    // Background layer colors
    const SKY_TOP = '#4a90c4';
    const SKY_BOTTOM = '#87CEEB';
    const MOUNTAIN_FAR = '#6b7b8a';
    const MOUNTAIN_MID = '#4a6548';
    const MOUNTAIN_NEAR = '#3d5a3a';
    const TREE_COLOR = '#2d5a2d';
    const TREE_DARK = '#1a4020';
    const TREE_TRUNK = '#4a3020';

    function create() {
        const terrain = {
            segments: [],      // Ground physics bodies
            rocks: [],         // Obstacle bodies
            roots: [],         // Root obstacles
            decorations: [],   // Visual-only elements (trees, bushes, etc.)
            generated: 0,      // How far we've generated (in pixels)
            groundPoints: [],  // Surface points for rendering smooth trail
            trees: [],         // Background trees
            mountains: [],     // Mountain silhouette points
        };

        // Pre-generate mountain silhouettes
        terrain.mountains = generateMountains();

        return terrain;
    }

    function generateMountains() {
        const layers = [];
        // Mountains need to cover the full course (100 miles * 2000 px/mi = 200k px)
        // But with parallax, they move slower, so we need less.
        // At parallax 0.05, camera at 200k means mountain offset = 10k, so 15k of points is enough.
        // At parallax 0.2, camera at 200k means mountain offset = 40k, so 50k of points is enough.
        // In practice, nobody will get past a few hundred pixels, so 50k is overkill but safe.

        // Far mountains
        const far = [];
        for (let x = -500; x < 15000; x += 80) {
            const h = 120 + Math.sin(x * 0.002) * 60 + Math.sin(x * 0.005) * 30 +
                      Math.sin(x * 0.001) * 40;
            far.push({ x, y: BASE_GROUND_Y - 200 - h });
        }
        layers.push({ points: far, color: MOUNTAIN_FAR, parallax: 0.05 });

        // Mid mountains
        const mid = [];
        for (let x = -500; x < 30000; x += 50) {
            const h = 80 + Math.sin(x * 0.003 + 1) * 50 + Math.sin(x * 0.007) * 25;
            mid.push({ x, y: BASE_GROUND_Y - 150 - h });
        }
        layers.push({ points: mid, color: MOUNTAIN_MID, parallax: 0.1 });

        // Near hills / treeline
        const near = [];
        for (let x = -500; x < 50000; x += 30) {
            const h = 40 + Math.sin(x * 0.005 + 2) * 30 + Math.sin(x * 0.01) * 15;
            near.push({ x, y: BASE_GROUND_Y - 80 - h });
        }
        layers.push({ points: near, color: MOUNTAIN_NEAR, parallax: 0.2 });

        return layers;
    }

    // Generate terrain segments up to the given X position
    function generateTo(terrain, world, toX) {
        const startX = terrain.generated;
        if (toX <= startX) return;

        const groundCategory = 0x0001;
        const collisionFilter = {
            category: groundCategory,
            mask: 0xFFFF,
        };

        for (let x = startX; x < toX; x += SEGMENT_WIDTH) {
            // Calculate ground height with gentle undulations
            const noise1 = Math.sin(x * 0.008) * TERRAIN_AMPLITUDE * 0.6;
            const noise2 = Math.sin(x * 0.003 + 1.5) * TERRAIN_AMPLITUDE * 0.8;
            const noise3 = Math.sin(x * 0.015) * TERRAIN_AMPLITUDE * 0.3;
            const groundY = BASE_GROUND_Y + noise1 + noise2 + noise3;

            // Next segment's height for angling
            const nextX = x + SEGMENT_WIDTH;
            const nextNoise1 = Math.sin(nextX * 0.008) * TERRAIN_AMPLITUDE * 0.6;
            const nextNoise2 = Math.sin(nextX * 0.003 + 1.5) * TERRAIN_AMPLITUDE * 0.8;
            const nextNoise3 = Math.sin(nextX * 0.015) * TERRAIN_AMPLITUDE * 0.3;
            const nextGroundY = BASE_GROUND_Y + nextNoise1 + nextNoise2 + nextNoise3;

            // Calculate angle for this segment
            const angle = Math.atan2(nextGroundY - groundY, SEGMENT_WIDTH);
            const midX = x + SEGMENT_WIDTH / 2;
            const midY = (groundY + nextGroundY) / 2 + GROUND_THICKNESS / 2;

            // Create angled ground segment
            const segment = Bodies.rectangle(midX, midY, SEGMENT_WIDTH + 2, GROUND_THICKNESS, {
                isStatic: true,
                angle: angle,
                friction: 0.8,
                restitution: 0.05,
                collisionFilter: collisionFilter,
                render: { visible: false },
                label: 'ground',
            });

            terrain.segments.push(segment);
            Composite.add(world, segment);

            // Store surface point for rendering
            terrain.groundPoints.push({
                x: x,
                y: groundY,
            });

            // Randomly place rocks
            if (Math.random() < ROCK_CHANCE && x > 300) {
                const rockSize = 4 + Math.random() * 8;
                const rock = Bodies.circle(
                    x + Math.random() * SEGMENT_WIDTH,
                    groundY - rockSize / 2,
                    rockSize,
                    {
                        isStatic: true,
                        friction: 0.5,
                        restitution: 0.3,
                        collisionFilter: collisionFilter,
                        label: 'rock',
                    }
                );
                terrain.rocks.push(rock);
                Composite.add(world, rock);
            }

            // Randomly place roots (small bumps)
            if (Math.random() < ROOT_CHANCE && x > 300) {
                const root = Bodies.rectangle(
                    x + Math.random() * SEGMENT_WIDTH,
                    groundY - 2,
                    12 + Math.random() * 8,
                    4,
                    {
                        isStatic: true,
                        friction: 0.4,
                        angle: (Math.random() - 0.5) * 0.3,
                        collisionFilter: collisionFilter,
                        label: 'root',
                    }
                );
                terrain.roots.push(root);
                Composite.add(world, root);
            }

            // Place trees in background (visual only)
            if (Math.random() < 0.15) {
                terrain.trees.push({
                    x: x + Math.random() * SEGMENT_WIDTH,
                    y: groundY,
                    height: 60 + Math.random() * 80,
                    width: 15 + Math.random() * 15,
                    layer: Math.random() < 0.5 ? 'back' : 'front',
                });
            }
        }

        // Add a final surface point
        const finalX = toX;
        const fn1 = Math.sin(finalX * 0.008) * TERRAIN_AMPLITUDE * 0.6;
        const fn2 = Math.sin(finalX * 0.003 + 1.5) * TERRAIN_AMPLITUDE * 0.8;
        const fn3 = Math.sin(finalX * 0.015) * TERRAIN_AMPLITUDE * 0.3;
        terrain.groundPoints.push({ x: finalX, y: BASE_GROUND_Y + fn1 + fn2 + fn3 });

        terrain.generated = toX;
    }

    // Get the ground Y at a given X position
    function getGroundY(x) {
        const noise1 = Math.sin(x * 0.008) * TERRAIN_AMPLITUDE * 0.6;
        const noise2 = Math.sin(x * 0.003 + 1.5) * TERRAIN_AMPLITUDE * 0.8;
        const noise3 = Math.sin(x * 0.015) * TERRAIN_AMPLITUDE * 0.3;
        return BASE_GROUND_Y + noise1 + noise2 + noise3;
    }

    // Convert pixel distance to miles
    function pixelsToMiles(pixels) {
        return pixels / PIXELS_PER_MILE;
    }

    // Get current mile marker info
    function getCurrentMarker(miles) {
        let current = MILE_MARKERS[0];
        for (let i = MILE_MARKERS.length - 1; i >= 0; i--) {
            if (miles >= MILE_MARKERS[i].mile) {
                current = MILE_MARKERS[i];
                break;
            }
        }
        return current;
    }

    // Get next mile marker
    function getNextMarker(miles) {
        for (let i = 0; i < MILE_MARKERS.length; i++) {
            if (MILE_MARKERS[i].mile > miles) {
                return MILE_MARKERS[i];
            }
        }
        return MILE_MARKERS[MILE_MARKERS.length - 1];
    }

    // Render the terrain and background
    function render(ctx, terrain, cameraX, cameraY, canvasWidth, canvasHeight) {
        // --- SKY ---
        const skyGrad = ctx.createLinearGradient(0, 0, 0, canvasHeight);
        skyGrad.addColorStop(0, SKY_TOP);
        skyGrad.addColorStop(0.6, SKY_BOTTOM);
        skyGrad.addColorStop(1, '#c9dea0');
        ctx.fillStyle = skyGrad;
        ctx.fillRect(0, 0, canvasWidth, canvasHeight);

        // --- SUN ---
        const sunX = canvasWidth * 0.8;
        const sunY = 80;
        ctx.fillStyle = '#fff4c4';
        ctx.beginPath();
        ctx.arc(sunX, sunY, 40, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillStyle = 'rgba(255,244,196,0.3)';
        ctx.beginPath();
        ctx.arc(sunX, sunY, 60, 0, Math.PI * 2);
        ctx.fill();

        // --- MOUNTAIN LAYERS ---
        terrain.mountains.forEach(layer => {
            renderMountainLayer(ctx, layer, cameraX, canvasWidth, canvasHeight);
        });

        // --- BACK TREES ---
        terrain.trees.forEach(tree => {
            if (tree.layer === 'back') {
                renderTree(ctx, tree, cameraX * 0.4, cameraY * 0.3, 0.6);
            }
        });

        // --- TRAIL SURFACE ---
        renderTrail(ctx, terrain, cameraX, cameraY, canvasWidth, canvasHeight);

        // --- RIVER CROSSING (Mile 78) ---
        renderRiverCrossing(ctx, cameraX, cameraY, canvasWidth, canvasHeight);

        // --- ROCKS ---
        terrain.rocks.forEach(rock => {
            const sx = rock.position.x - cameraX;
            const sy = rock.position.y - cameraY;
            if (sx > -50 && sx < canvasWidth + 50) {
                const r = rock.circleRadius || 6;
                ctx.fillStyle = ROCK_COLOR;
                ctx.beginPath();
                ctx.arc(sx, sy, r, 0, Math.PI * 2);
                ctx.fill();
                ctx.fillStyle = ROCK_DARK;
                ctx.beginPath();
                ctx.arc(sx - r * 0.2, sy - r * 0.2, r * 0.7, 0, Math.PI * 2);
                ctx.fill();
            }
        });

        // --- ROOTS ---
        terrain.roots.forEach(root => {
            const sx = root.position.x - cameraX;
            const sy = root.position.y - cameraY;
            if (sx > -50 && sx < canvasWidth + 50) {
                ctx.save();
                ctx.translate(sx, sy);
                ctx.rotate(root.angle);
                ctx.fillStyle = ROOT_COLOR;
                ctx.beginPath();
                ctx.ellipse(0, 0, 8, 2.5, 0, 0, Math.PI * 2);
                ctx.fill();
                ctx.restore();
            }
        });

        // --- FRONT TREES ---
        terrain.trees.forEach(tree => {
            if (tree.layer === 'front') {
                renderTree(ctx, tree, cameraX * 0.7, cameraY * 0.5, 0.85);
            }
        });

        // --- MILE MARKERS ---
        renderMileMarkers(ctx, terrain, cameraX, cameraY, canvasWidth);
    }

    function renderMountainLayer(ctx, layer, cameraX, canvasWidth, canvasHeight) {
        const offsetX = cameraX * layer.parallax;
        ctx.fillStyle = layer.color;
        ctx.beginPath();

        let started = false;
        for (let i = 0; i < layer.points.length; i++) {
            const sx = layer.points[i].x - offsetX;
            if (sx < -200 || sx > canvasWidth + 200) continue;

            if (!started) {
                ctx.moveTo(sx, layer.points[i].y);
                started = true;
            } else {
                ctx.lineTo(sx, layer.points[i].y);
            }
        }

        ctx.lineTo(canvasWidth + 200, canvasHeight);
        ctx.lineTo(-200, canvasHeight);
        ctx.closePath();
        ctx.fill();
    }

    function renderTree(ctx, tree, offsetX, offsetY, scale) {
        const sx = tree.x - offsetX;
        const sy = tree.y - offsetY;
        const h = tree.height * scale;
        const w = tree.width * scale;

        // Pine tree shape
        ctx.fillStyle = TREE_TRUNK;
        ctx.fillRect(sx - 3 * scale, sy - 5, 6 * scale, 15 * scale);

        ctx.fillStyle = TREE_DARK;
        // Triangle layers
        for (let i = 0; i < 3; i++) {
            const layerW = w * (1 - i * 0.2);
            const layerY = sy - h * 0.3 - i * h * 0.25;
            ctx.beginPath();
            ctx.moveTo(sx, layerY - h * 0.3);
            ctx.lineTo(sx - layerW, layerY + h * 0.15);
            ctx.lineTo(sx + layerW, layerY + h * 0.15);
            ctx.closePath();
            ctx.fill();
        }

        // Lighter front
        ctx.fillStyle = TREE_COLOR;
        for (let i = 0; i < 3; i++) {
            const layerW = w * 0.7 * (1 - i * 0.2);
            const layerY = sy - h * 0.3 - i * h * 0.25 + 3;
            ctx.beginPath();
            ctx.moveTo(sx + 2, layerY - h * 0.25);
            ctx.lineTo(sx - layerW + 2, layerY + h * 0.12);
            ctx.lineTo(sx + layerW + 2, layerY + h * 0.12);
            ctx.closePath();
            ctx.fill();
        }
    }

    function renderTrail(ctx, terrain, cameraX, cameraY, canvasWidth, canvasHeight) {
        const points = terrain.groundPoints;
        if (points.length < 2) return;

        // Find visible range
        let startIdx = 0;
        let endIdx = points.length - 1;
        for (let i = 0; i < points.length; i++) {
            if (points[i].x - cameraX > -SEGMENT_WIDTH * 2) {
                startIdx = Math.max(0, i - 1);
                break;
            }
        }
        for (let i = points.length - 1; i >= 0; i--) {
            if (points[i].x - cameraX < canvasWidth + SEGMENT_WIDTH * 2) {
                endIdx = Math.min(points.length - 1, i + 1);
                break;
            }
        }

        // Draw trail surface (dirt)
        ctx.fillStyle = TRAIL_COLOR;
        ctx.beginPath();
        ctx.moveTo(points[startIdx].x - cameraX, points[startIdx].y - cameraY);

        for (let i = startIdx; i <= endIdx; i++) {
            ctx.lineTo(points[i].x - cameraX, points[i].y - cameraY);
        }

        ctx.lineTo(points[endIdx].x - cameraX, canvasHeight + 100);
        ctx.lineTo(points[startIdx].x - cameraX, canvasHeight + 100);
        ctx.closePath();
        ctx.fill();

        // Trail surface highlight (lighter dirt on top)
        ctx.strokeStyle = TRAIL_SURFACE;
        ctx.lineWidth = 4;
        ctx.beginPath();
        for (let i = startIdx; i <= endIdx; i++) {
            const sx = points[i].x - cameraX;
            const sy = points[i].y - cameraY;
            if (i === startIdx) ctx.moveTo(sx, sy);
            else ctx.lineTo(sx, sy);
        }
        ctx.stroke();

        // Dirt texture lines underneath
        ctx.strokeStyle = DIRT_DARK;
        ctx.lineWidth = 1;
        for (let depth = 10; depth < 200; depth += 20) {
            ctx.globalAlpha = 0.2;
            ctx.beginPath();
            for (let i = startIdx; i <= endIdx; i++) {
                const sx = points[i].x - cameraX;
                const sy = points[i].y - cameraY + depth + Math.sin(points[i].x * 0.02 + depth) * 3;
                if (i === startIdx) ctx.moveTo(sx, sy);
                else ctx.lineTo(sx, sy);
            }
            ctx.stroke();
        }
        ctx.globalAlpha = 1.0;

        // Small pebbles/texture on trail surface
        ctx.fillStyle = 'rgba(120,100,70,0.4)';
        for (let i = startIdx; i <= endIdx; i += 2) {
            const px = points[i].x - cameraX + ((points[i].x * 7) % 30) - 15;
            const py = points[i].y - cameraY - 1 + ((points[i].x * 13) % 4);
            ctx.beginPath();
            ctx.arc(px, py, 1 + ((points[i].x * 3) % 2), 0, Math.PI * 2);
            ctx.fill();
        }
    }

    function renderRiverCrossing(ctx, cameraX, cameraY, canvasWidth, canvasHeight) {
        // River crossing at mile 78
        const riverCenterX = 78 * PIXELS_PER_MILE;
        const riverWidth = 300; // pixels wide
        const riverLeft = riverCenterX - riverWidth / 2;
        const riverRight = riverCenterX + riverWidth / 2;

        const screenLeft = riverLeft - cameraX;
        const screenRight = riverRight - cameraX;

        // Only render if visible
        if (screenRight < -50 || screenLeft > canvasWidth + 50) return;

        const riverY = getGroundY(riverCenterX) - cameraY;

        // Water body
        ctx.fillStyle = 'rgba(60, 130, 180, 0.6)';
        ctx.fillRect(screenLeft, riverY - 5, riverWidth, 40);

        // Water surface shimmer
        ctx.strokeStyle = 'rgba(180, 220, 255, 0.5)';
        ctx.lineWidth = 1;
        const time = Date.now() * 0.002;
        for (let i = 0; i < 8; i++) {
            ctx.beginPath();
            const waveY = riverY + i * 4 + Math.sin(time + i) * 2;
            ctx.moveTo(screenLeft, waveY);
            for (let x = screenLeft; x < screenRight; x += 10) {
                ctx.lineTo(x, waveY + Math.sin(time + x * 0.05 + i) * 2);
            }
            ctx.stroke();
        }
    }

    function renderMileMarkers(ctx, terrain, cameraX, cameraY, canvasWidth) {
        MILE_MARKERS.forEach(marker => {
            const markerX = marker.mile * PIXELS_PER_MILE;
            const sx = markerX - cameraX;

            if (sx > -100 && sx < canvasWidth + 100) {
                const groundY = getGroundY(markerX) - cameraY;

                // Post
                ctx.fillStyle = '#c9b896';
                ctx.fillRect(sx - 2, groundY - 50, 4, 50);

                // Sign
                ctx.fillStyle = '#f0e6d3';
                ctx.fillRect(sx - 40, groundY - 65, 80, 22);
                ctx.strokeStyle = '#8a7a6a';
                ctx.lineWidth = 1;
                ctx.strokeRect(sx - 40, groundY - 65, 80, 22);

                // Text
                ctx.fillStyle = '#2a1f14';
                ctx.font = 'bold 9px Courier New';
                ctx.textAlign = 'center';
                ctx.fillText(`Mile ${marker.mile}`, sx, groundY - 56);
                ctx.font = '7px Courier New';
                ctx.fillText(marker.name, sx, groundY - 48);
            }
        });
    }

    // Check if the runner has reached the finish
    function isFinished(distancePixels) {
        return distancePixels >= 100 * PIXELS_PER_MILE;
    }

    return {
        create,
        generateTo,
        getGroundY,
        pixelsToMiles,
        getCurrentMarker,
        getNextMarker,
        render,
        isFinished,
        PIXELS_PER_MILE,
        BASE_GROUND_Y,
        MILE_MARKERS,
    };
})();
