/**
 * JREK Runner - Ragdoll physics body for Scott Jurek
 * Built with Matter.js rigid bodies and constraints
 *
 * Physics approach: Strong angular springs keep the runner standing.
 * Controls apply forces that OVERRIDE the springs to create movement.
 * The challenge is coordinating the override to produce a running gait.
 */

const Runner = (function() {
    const { Bodies, Body, Composite, Constraint, Vector } = Matter;

    // Body part dimensions
    const SCALE = 1.0;
    const HEAD_RADIUS = 12 * SCALE;
    const TORSO_W = 18 * SCALE;
    const TORSO_H = 42 * SCALE;
    const UPPER_LEG_W = 11 * SCALE;
    const UPPER_LEG_H = 34 * SCALE;
    const LOWER_LEG_W = 9 * SCALE;
    const LOWER_LEG_H = 32 * SCALE;
    const FOOT_W = 22 * SCALE;
    const FOOT_H = 7 * SCALE;

    // Physics tuning
    const BODY_FRICTION = 0.8;
    const BODY_RESTITUTION = 0.02;

    // Densities - heavier feet and torso for stability
    const LIMB_DENSITY = 0.0008;
    const TORSO_DENSITY = 0.004;
    const HEAD_DENSITY = 0.001;
    const FOOT_DENSITY = 0.005;

    // Joint stiffness (constraint stiffness)
    const JOINT_STIFFNESS = 1.0;
    const JOINT_DAMPING = 0.5;

    // Control force magnitudes - STRONG so they create visible movement
    const HIP_FORCE = 0.004;    // Force applied at thigh center of mass
    const KNEE_FORCE = 0.003;   // Force applied at calf center of mass
    const HIP_TORQUE = 0.12;    // Angular torque on thighs
    const KNEE_TORQUE = 0.08;   // Angular torque on calves

    // Collision categories
    const RUNNER_CATEGORY = 0x0002;
    const GROUND_CATEGORY = 0x0001;

    function create(x, y) {
        const parts = {};
        const constraints = [];

        const collisionFilter = {
            category: RUNNER_CATEGORY,
            mask: GROUND_CATEGORY,
        };

        const bodyOptions = {
            friction: BODY_FRICTION,
            restitution: BODY_RESTITUTION,
            collisionFilter: collisionFilter,
            frictionAir: 0.02,
        };

        // HEAD
        parts.head = Bodies.circle(x, y - TORSO_H / 2 - HEAD_RADIUS - 2, HEAD_RADIUS, {
            ...bodyOptions,
            density: HEAD_DENSITY,
            label: 'head',
        });

        // TORSO
        parts.torso = Bodies.rectangle(x, y, TORSO_W, TORSO_H, {
            ...bodyOptions,
            density: TORSO_DENSITY,
            label: 'torso',
            frictionAir: 0.03,
        });

        // LEFT UPPER LEG (thigh)
        parts.leftThigh = Bodies.rectangle(
            x - 3, y + TORSO_H / 2 + UPPER_LEG_H / 2,
            UPPER_LEG_W, UPPER_LEG_H,
            { ...bodyOptions, density: LIMB_DENSITY, label: 'leftThigh' }
        );

        // RIGHT UPPER LEG (thigh)
        parts.rightThigh = Bodies.rectangle(
            x + 3, y + TORSO_H / 2 + UPPER_LEG_H / 2,
            UPPER_LEG_W, UPPER_LEG_H,
            { ...bodyOptions, density: LIMB_DENSITY, label: 'rightThigh' }
        );

        // LEFT LOWER LEG (calf)
        parts.leftCalf = Bodies.rectangle(
            x - 3, y + TORSO_H / 2 + UPPER_LEG_H + LOWER_LEG_H / 2,
            LOWER_LEG_W, LOWER_LEG_H,
            { ...bodyOptions, density: LIMB_DENSITY, label: 'leftCalf' }
        );

        // RIGHT LOWER LEG (calf)
        parts.rightCalf = Bodies.rectangle(
            x + 3, y + TORSO_H / 2 + UPPER_LEG_H + LOWER_LEG_H / 2,
            LOWER_LEG_W, LOWER_LEG_H,
            { ...bodyOptions, density: LIMB_DENSITY, label: 'rightCalf' }
        );

        // LEFT FOOT
        parts.leftFoot = Bodies.rectangle(
            x - 3, y + TORSO_H / 2 + UPPER_LEG_H + LOWER_LEG_H + FOOT_H / 2,
            FOOT_W, FOOT_H,
            { ...bodyOptions, density: FOOT_DENSITY, friction: 1.0, label: 'leftFoot' }
        );

        // RIGHT FOOT
        parts.rightFoot = Bodies.rectangle(
            x + 3, y + TORSO_H / 2 + UPPER_LEG_H + LOWER_LEG_H + FOOT_H / 2,
            FOOT_W, FOOT_H,
            { ...bodyOptions, density: FOOT_DENSITY, friction: 1.0, label: 'rightFoot' }
        );

        // === CONSTRAINTS ===

        // Neck: head to torso
        constraints.push(Constraint.create({
            bodyA: parts.head,
            pointA: { x: 0, y: HEAD_RADIUS * 0.8 },
            bodyB: parts.torso,
            pointB: { x: 0, y: -TORSO_H / 2 },
            stiffness: 0.9,
            damping: 0.5,
            length: 2,
            label: 'neck',
        }));

        // Head stabilizer (keeps head above torso)
        constraints.push(Constraint.create({
            bodyA: parts.head,
            pointA: { x: 0, y: 0 },
            bodyB: parts.torso,
            pointB: { x: 0, y: -TORSO_H / 2 + 5 },
            stiffness: 0.3,
            damping: 0.3,
            length: HEAD_RADIUS + 5,
            label: 'headStabilizer',
        }));

        // Left Hip
        constraints.push(Constraint.create({
            bodyA: parts.torso,
            pointA: { x: -3, y: TORSO_H / 2 },
            bodyB: parts.leftThigh,
            pointB: { x: 0, y: -UPPER_LEG_H / 2 },
            stiffness: JOINT_STIFFNESS,
            damping: JOINT_DAMPING,
            length: 0,
            label: 'leftHip',
        }));

        // Right Hip
        constraints.push(Constraint.create({
            bodyA: parts.torso,
            pointA: { x: 3, y: TORSO_H / 2 },
            bodyB: parts.rightThigh,
            pointB: { x: 0, y: -UPPER_LEG_H / 2 },
            stiffness: JOINT_STIFFNESS,
            damping: JOINT_DAMPING,
            length: 0,
            label: 'rightHip',
        }));

        // Left Knee
        constraints.push(Constraint.create({
            bodyA: parts.leftThigh,
            pointA: { x: 0, y: UPPER_LEG_H / 2 },
            bodyB: parts.leftCalf,
            pointB: { x: 0, y: -LOWER_LEG_H / 2 },
            stiffness: JOINT_STIFFNESS,
            damping: JOINT_DAMPING,
            length: 0,
            label: 'leftKnee',
        }));

        // Right Knee
        constraints.push(Constraint.create({
            bodyA: parts.rightThigh,
            pointA: { x: 0, y: UPPER_LEG_H / 2 },
            bodyB: parts.rightCalf,
            pointB: { x: 0, y: -LOWER_LEG_H / 2 },
            stiffness: JOINT_STIFFNESS,
            damping: JOINT_DAMPING,
            length: 0,
            label: 'rightKnee',
        }));

        // Left Ankle
        constraints.push(Constraint.create({
            bodyA: parts.leftCalf,
            pointA: { x: 0, y: LOWER_LEG_H / 2 },
            bodyB: parts.leftFoot,
            pointB: { x: -2, y: 0 },
            stiffness: 0.8,
            damping: 0.4,
            length: 0,
            label: 'leftAnkle',
        }));

        // Right Ankle
        constraints.push(Constraint.create({
            bodyA: parts.rightCalf,
            pointA: { x: 0, y: LOWER_LEG_H / 2 },
            bodyB: parts.rightFoot,
            pointB: { x: -2, y: 0 },
            stiffness: 0.8,
            damping: 0.4,
            length: 0,
            label: 'rightAnkle',
        }));

        // === STRUCTURAL BRACES ===

        // Torso-to-foot distance constraints (prevents legs from collapsing)
        constraints.push(Constraint.create({
            bodyA: parts.torso,
            pointA: { x: -3, y: TORSO_H / 2 },
            bodyB: parts.leftFoot,
            pointB: { x: 0, y: 0 },
            stiffness: 0.08,
            damping: 0.2,
            length: UPPER_LEG_H + LOWER_LEG_H + FOOT_H / 2,
            label: 'leftFullLeg',
        }));

        constraints.push(Constraint.create({
            bodyA: parts.torso,
            pointA: { x: 3, y: TORSO_H / 2 },
            bodyB: parts.rightFoot,
            pointB: { x: 0, y: 0 },
            stiffness: 0.08,
            damping: 0.2,
            length: UPPER_LEG_H + LOWER_LEG_H + FOOT_H / 2,
            label: 'rightFullLeg',
        }));

        // Thigh cross-brace (prevents splits)
        constraints.push(Constraint.create({
            bodyA: parts.leftThigh,
            pointA: { x: 0, y: UPPER_LEG_H / 3 },
            bodyB: parts.rightThigh,
            pointB: { x: 0, y: UPPER_LEG_H / 3 },
            stiffness: 0.15,
            damping: 0.3,
            length: 10,
            label: 'thighBrace',
        }));

        const runner = {
            parts,
            constraints,
            startX: x,
            fallen: false,
            fallTime: 0,
            distance: 0,
            maxDistance: 0,
        };

        return runner;
    }

    function addToWorld(world, runner) {
        const allBodies = Object.values(runner.parts);
        Composite.add(world, allBodies);
        Composite.add(world, runner.constraints);
    }

    function removeFromWorld(world, runner) {
        const allBodies = Object.values(runner.parts);
        Composite.remove(world, allBodies);
        Composite.remove(world, runner.constraints);
    }

    /**
     * Core stabilization system.
     * Uses BOTH angular velocity correction AND positional forces.
     * The angular springs keep limbs at target angles.
     * When no keys are pressed, the target is "standing straight."
     */
    function stabilize(runner, keys) {
        const { torso, leftThigh, rightThigh, leftCalf, rightCalf, leftFoot, rightFoot } = runner.parts;

        const anyKeyPressed = keys && (keys.j || keys.r || keys.e || keys.k);
        // Reduce stabilization strength when keys are pressed so controls can override
        const stabStrength = anyKeyPressed ? 0.4 : 1.0;

        // === TORSO: strong uprighting ===
        const torsoAngleError = torso.angle; // target = 0 (vertical)
        Body.setAngularVelocity(torso,
            torso.angularVelocity * 0.7 - torsoAngleError * 0.18 * stabStrength
        );

        // === THIGHS: want to point straight down (angle = torso.angle) ===
        const leftThighError = leftThigh.angle - torso.angle;
        const rightThighError = rightThigh.angle - torso.angle;

        // Strong angular correction
        Body.setAngularVelocity(leftThigh,
            leftThigh.angularVelocity * 0.7 - leftThighError * 0.14 * stabStrength
        );
        Body.setAngularVelocity(rightThigh,
            rightThigh.angularVelocity * 0.7 - rightThighError * 0.14 * stabStrength
        );

        // Also apply a POSITIONAL force pushing thighs downward when they drift
        // This is key -- angular velocity alone isn't enough
        if (Math.abs(leftThighError) > 0.1) {
            Body.applyForce(leftThigh, leftThigh.position, {
                x: -Math.sin(leftThighError) * 0.0008 * stabStrength,
                y: Math.abs(Math.cos(leftThighError)) * 0.0003 * stabStrength
            });
        }
        if (Math.abs(rightThighError) > 0.1) {
            Body.applyForce(rightThigh, rightThigh.position, {
                x: -Math.sin(rightThighError) * 0.0008 * stabStrength,
                y: Math.abs(Math.cos(rightThighError)) * 0.0003 * stabStrength
            });
        }

        // === CALVES: want to stay in line with their thigh (straight leg) ===
        const leftKneeError = leftCalf.angle - leftThigh.angle;
        const rightKneeError = rightCalf.angle - rightThigh.angle;

        Body.setAngularVelocity(leftCalf,
            leftCalf.angularVelocity * 0.7 - leftKneeError * 0.12 * stabStrength
        );
        Body.setAngularVelocity(rightCalf,
            rightCalf.angularVelocity * 0.7 - rightKneeError * 0.12 * stabStrength
        );

        // === FEET: want to stay flat (angle = 0) ===
        Body.setAngularVelocity(leftFoot,
            leftFoot.angularVelocity * 0.5 - leftFoot.angle * 0.08
        );
        Body.setAngularVelocity(rightFoot,
            rightFoot.angularVelocity * 0.5 - rightFoot.angle * 0.08
        );
    }

    /**
     * Apply control forces.
     * J/R swing the thighs forward (like kicking). These are the primary movers.
     * E/K extend the calves (push off the ground). These provide thrust.
     *
     * The combination of swinging a thigh forward while extending the opposite
     * calf is what creates forward motion (like actual running).
     */
    function applyControls(runner, keys) {
        const { torso, leftThigh, rightThigh, leftCalf, rightCalf } = runner.parts;

        // Run stabilization (with awareness of which keys are pressed)
        stabilize(runner, keys);

        // J key: LEFT thigh swings forward, RIGHT thigh pushes back
        if (keys.j) {
            // Swing left thigh forward (clockwise rotation)
            Body.setAngularVelocity(leftThigh, leftThigh.angularVelocity + HIP_TORQUE);
            // Also apply a forward force at the thigh
            Body.applyForce(leftThigh, leftThigh.position, { x: HIP_FORCE, y: -HIP_FORCE * 0.3 });
            // Push right thigh back
            Body.setAngularVelocity(rightThigh, rightThigh.angularVelocity - HIP_TORQUE * 0.6);
            Body.applyForce(rightThigh, rightThigh.position, { x: -HIP_FORCE * 0.4, y: 0 });
            // Slight lean forward
            Body.setAngularVelocity(torso, torso.angularVelocity + 0.008);
        }

        // R key: RIGHT thigh swings forward, LEFT thigh pushes back
        if (keys.r) {
            Body.setAngularVelocity(rightThigh, rightThigh.angularVelocity + HIP_TORQUE);
            Body.applyForce(rightThigh, rightThigh.position, { x: HIP_FORCE, y: -HIP_FORCE * 0.3 });
            Body.setAngularVelocity(leftThigh, leftThigh.angularVelocity - HIP_TORQUE * 0.6);
            Body.applyForce(leftThigh, leftThigh.position, { x: -HIP_FORCE * 0.4, y: 0 });
            Body.setAngularVelocity(torso, torso.angularVelocity + 0.008);
        }

        // E key: LEFT calf kicks out (extends knee) - provides push-off
        if (keys.e) {
            Body.setAngularVelocity(leftCalf, leftCalf.angularVelocity - KNEE_TORQUE);
            Body.applyForce(leftCalf, leftCalf.position, { x: KNEE_FORCE, y: -KNEE_FORCE * 0.5 });
            // Reaction on thigh
            Body.setAngularVelocity(leftThigh, leftThigh.angularVelocity + KNEE_TORQUE * 0.3);
        }

        // K key: RIGHT calf kicks out
        if (keys.k) {
            Body.setAngularVelocity(rightCalf, rightCalf.angularVelocity - KNEE_TORQUE);
            Body.applyForce(rightCalf, rightCalf.position, { x: KNEE_FORCE, y: -KNEE_FORCE * 0.5 });
            Body.setAngularVelocity(rightThigh, rightThigh.angularVelocity + KNEE_TORQUE * 0.3);
        }
    }

    // Check if the runner has fallen
    function checkFallen(runner, groundY) {
        const head = runner.parts.head;
        const torso = runner.parts.torso;

        const torsoAngle = Math.abs(torso.angle);
        const headLow = head.position.y > groundY - 15;
        const torsoTilted = torsoAngle > 1.4; // ~80 degrees

        if ((headLow || torsoTilted) && !runner.fallen) {
            runner.fallen = true;
            runner.fallTime = Date.now();
            return true;
        }
        return false;
    }

    function updateDistance(runner) {
        const torsoX = runner.parts.torso.position.x;
        runner.distance = Math.max(0, torsoX - runner.startX);
        runner.maxDistance = Math.max(runner.maxDistance, runner.distance);
        return runner.distance;
    }

    function getCenter(runner) {
        return {
            x: runner.parts.torso.position.x,
            y: runner.parts.torso.position.y,
        };
    }

    // Draw the runner on canvas - Classic Jurek Western States look
    // Reference: White Montrail/Patagonia crop top, CLIF cap, black shorts,
    // handheld bottle, long hair, clean-shaven, yellow wristband
    function render(ctx, runner, cameraX, cameraY) {
        const parts = runner.parts;
        const ox = -cameraX;
        const oy = -cameraY;

        ctx.save();
        ctx.translate(ox, oy);

        // Color palette - Jurek at Western States circa 2003-2005
        const skinColor = '#c49a6c';
        const skinShadow = '#a8825a';
        const cropTopColor = '#f0ebe3'; // White crop top
        const cropTopShadow = '#d8d0c4';
        const shortsColor = '#1a1a1a'; // Black shorts
        const gearBeltColor = '#333333'; // Gear belt/pack
        const shoeColor = '#4a6741'; // Montrail green trail shoes
        const shoeSoleColor = '#1a1a1a';
        const capRedColor = '#cc2222'; // CLIF red cap panel
        const capWhiteColor = '#e8e0d4'; // Cap mesh
        const capBrimColor = '#2a2a2a';
        const hairColor = '#3b2507'; // Long brown hair
        const wristbandColor = '#e6cc00'; // Yellow wristband
        const watchColor = '#222222';
        const bottleColor = '#e8e8e8'; // White handheld bottle

        // Helper: draw a rotated rectangle
        function drawBodyPart(body, w, h, color) {
            ctx.save();
            ctx.translate(body.position.x, body.position.y);
            ctx.rotate(body.angle);
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.roundRect(-w / 2, -h / 2, w, h, 3);
            ctx.fill();
            ctx.restore();
        }

        // Helper: draw a shoe
        function drawShoe(foot, isBack) {
            ctx.save();
            ctx.translate(foot.position.x, foot.position.y);
            ctx.rotate(foot.angle);
            // Shoe body
            ctx.fillStyle = shoeColor;
            ctx.beginPath();
            ctx.roundRect(-FOOT_W / 2, -FOOT_H / 2, FOOT_W, FOOT_H, 3);
            ctx.fill();
            // Sole
            ctx.fillStyle = shoeSoleColor;
            ctx.fillRect(-FOOT_W / 2, FOOT_H / 2 - 2, FOOT_W, 2);
            // Lace detail
            if (!isBack) {
                ctx.strokeStyle = '#cccccc';
                ctx.lineWidth = 0.7;
                ctx.beginPath();
                ctx.moveTo(-3, -FOOT_H / 2 + 1.5);
                ctx.lineTo(3, -FOOT_H / 2 + 1.5);
                ctx.stroke();
            }
            ctx.restore();
        }

        // === DRAW ORDER: back leg, back arm, torso, front arm, front leg, head ===

        // -- BACK LEG (left, slightly darker) --
        drawBodyPart(parts.leftThigh, UPPER_LEG_W, UPPER_LEG_H, skinShadow);
        drawBodyPart(parts.leftCalf, LOWER_LEG_W, LOWER_LEG_H, skinShadow);
        drawShoe(parts.leftFoot, true);

        // -- TORSO --
        ctx.save();
        ctx.translate(parts.torso.position.x, parts.torso.position.y);
        ctx.rotate(parts.torso.angle);

        // Full torso skin
        ctx.fillStyle = skinColor;
        ctx.beginPath();
        ctx.roundRect(-TORSO_W / 2, -TORSO_H / 2, TORSO_W, TORSO_H, 4);
        ctx.fill();

        // White crop top (upper ~50% of torso, showing midriff)
        const cropHeight = TORSO_H * 0.50;
        ctx.fillStyle = cropTopColor;
        ctx.beginPath();
        ctx.roundRect(-TORSO_W / 2 - 1, -TORSO_H / 2, TORSO_W + 2, cropHeight, [4, 4, 0, 0]);
        ctx.fill();

        // Crop top wrinkle/shadow lines for texture
        ctx.strokeStyle = cropTopShadow;
        ctx.lineWidth = 0.6;
        ctx.beginPath();
        ctx.moveTo(-TORSO_W / 4, -TORSO_H / 2 + cropHeight * 0.3);
        ctx.lineTo(TORSO_W / 4, -TORSO_H / 2 + cropHeight * 0.35);
        ctx.moveTo(-TORSO_W / 3, -TORSO_H / 2 + cropHeight * 0.6);
        ctx.lineTo(TORSO_W / 5, -TORSO_H / 2 + cropHeight * 0.55);
        ctx.stroke();

        // Crop top hem (slightly ragged/loose)
        ctx.strokeStyle = '#c0b8aa';
        ctx.lineWidth = 1.2;
        ctx.beginPath();
        ctx.moveTo(-TORSO_W / 2 - 1, -TORSO_H / 2 + cropHeight);
        ctx.lineTo(TORSO_W / 2 + 1, -TORSO_H / 2 + cropHeight + 1);
        ctx.stroke();

        // Exposed midriff area (skin is already drawn)

        // Black shorts with gear belt
        const shortsTop = TORSO_H / 4;
        ctx.fillStyle = shortsColor;
        ctx.beginPath();
        ctx.roundRect(-TORSO_W / 2 - 1, shortsTop, TORSO_W + 2, TORSO_H / 2 - shortsTop + 1, [0, 0, 3, 3]);
        ctx.fill();

        // Gear belt at top of shorts
        ctx.fillStyle = gearBeltColor;
        ctx.fillRect(-TORSO_W / 2 - 2, shortsTop - 1, TORSO_W + 4, 4);
        // Small gear pouch on the belt
        ctx.fillStyle = '#444444';
        ctx.fillRect(TORSO_W / 2 - 2, shortsTop - 2, 5, 6);

        ctx.restore();

        // -- FRONT LEG (right, brighter) --
        drawBodyPart(parts.rightThigh, UPPER_LEG_W, UPPER_LEG_H, skinColor);
        drawBodyPart(parts.rightCalf, LOWER_LEG_W, LOWER_LEG_H, skinColor);
        drawShoe(parts.rightFoot, false);

        // -- HANDHELD BOTTLE (attached to right hand area, near torso bottom) --
        ctx.save();
        ctx.translate(parts.torso.position.x, parts.torso.position.y);
        ctx.rotate(parts.torso.angle);
        // Bottle in right hand
        ctx.fillStyle = bottleColor;
        ctx.beginPath();
        ctx.roundRect(TORSO_W / 2 - 2, -2, 5, 10, 2);
        ctx.fill();
        // Bottle cap
        ctx.fillStyle = '#aaaaaa';
        ctx.beginPath();
        ctx.arc(TORSO_W / 2 + 0.5, -2, 2.5, Math.PI, 0);
        ctx.fill();
        // Yellow wristband on the same side
        ctx.fillStyle = wristbandColor;
        ctx.fillRect(TORSO_W / 2 - 3, 8, 7, 3);
        // Watch on left wrist area
        ctx.fillStyle = watchColor;
        ctx.fillRect(-TORSO_W / 2 - 3, 6, 5, 4);
        ctx.fillStyle = '#335533';
        ctx.fillRect(-TORSO_W / 2 - 2, 7, 3, 2);
        ctx.restore();

        // -- HEAD --
        ctx.save();
        ctx.translate(parts.head.position.x, parts.head.position.y);
        ctx.rotate(parts.head.angle);

        // Long hair flowing behind (Jurek's signature long hair)
        ctx.fillStyle = hairColor;
        ctx.beginPath();
        // Hair behind head
        ctx.arc(0, 0, HEAD_RADIUS + 2, Math.PI * 0.6, Math.PI * 1.4);
        // Long hair flowing down/back
        ctx.quadraticCurveTo(-HEAD_RADIUS - 4, HEAD_RADIUS + 6, -HEAD_RADIUS - 2, HEAD_RADIUS + 12);
        ctx.quadraticCurveTo(-HEAD_RADIUS + 2, HEAD_RADIUS + 10, -HEAD_RADIUS + 4, HEAD_RADIUS + 4);
        ctx.fill();

        // Head circle (skin)
        ctx.fillStyle = skinColor;
        ctx.beginPath();
        ctx.arc(0, 0, HEAD_RADIUS, 0, Math.PI * 2);
        ctx.fill();

        // CLIF trucker cap
        // Cap dome (red front panel)
        ctx.fillStyle = capRedColor;
        ctx.beginPath();
        ctx.arc(0, -2, HEAD_RADIUS + 2, Math.PI * 1.1, Math.PI * 1.7);
        ctx.lineTo(HEAD_RADIUS * 0.5, -HEAD_RADIUS - 3);
        ctx.quadraticCurveTo(0, -HEAD_RADIUS - 5, -HEAD_RADIUS * 0.5, -HEAD_RADIUS - 3);
        ctx.closePath();
        ctx.fill();

        // Cap mesh (white/grey back panels)
        ctx.fillStyle = capWhiteColor;
        ctx.beginPath();
        ctx.arc(0, -2, HEAD_RADIUS + 2, Math.PI * 1.7, Math.PI * 2.0);
        ctx.arc(0, -2, HEAD_RADIUS + 2, 0, Math.PI * 0.3);
        ctx.closePath();
        ctx.fill();

        // Cap brim
        ctx.fillStyle = capBrimColor;
        ctx.beginPath();
        ctx.moveTo(HEAD_RADIUS * 0.9, -HEAD_RADIUS * 0.3);
        ctx.quadraticCurveTo(HEAD_RADIUS + 8, -HEAD_RADIUS * 0.5, HEAD_RADIUS + 10, -HEAD_RADIUS * 0.15);
        ctx.quadraticCurveTo(HEAD_RADIUS + 8, HEAD_RADIUS * 0.1, HEAD_RADIUS * 0.7, -HEAD_RADIUS * 0.05);
        ctx.closePath();
        ctx.fill();

        // CLIF text on cap (tiny)
        ctx.fillStyle = '#ffffff';
        ctx.font = 'bold 5px Arial';
        ctx.textAlign = 'center';
        ctx.fillText('CLIF', 1, -HEAD_RADIUS + 1);

        // Eyes
        ctx.fillStyle = '#1a1208';
        ctx.beginPath();
        ctx.arc(-4, -1, 1.8, 0, Math.PI * 2);
        ctx.arc(4, -1, 1.8, 0, Math.PI * 2);
        ctx.fill();

        // Eyebrows (focused)
        ctx.strokeStyle = '#2a1a08';
        ctx.lineWidth = 1.3;
        ctx.beginPath();
        ctx.moveTo(-6, -4);
        ctx.lineTo(-2, -3.5);
        ctx.moveTo(2, -3.5);
        ctx.lineTo(6, -4);
        ctx.stroke();

        // Nose (subtle line)
        ctx.strokeStyle = '#a07850';
        ctx.lineWidth = 0.8;
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(1, 2);
        ctx.stroke();

        // Mouth
        ctx.strokeStyle = '#1a1208';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        if (runner.fallen) {
            // Yelling "MINNESOTA!!"
            ctx.arc(0, 5, 3.5, 0, Math.PI * 2);
            ctx.fillStyle = '#1a1208';
            ctx.fill();
        } else {
            // Determined grimace
            ctx.moveTo(-3, 5);
            ctx.lineTo(3, 5);
        }
        ctx.stroke();

        ctx.restore();

        // Joint dots (very subtle)
        ctx.fillStyle = 'rgba(0,0,0,0.15)';
        const jointSize = 2;
        drawJointDot(ctx, parts.torso, { x: -3, y: TORSO_H / 2 }, jointSize);
        drawJointDot(ctx, parts.torso, { x: 3, y: TORSO_H / 2 }, jointSize);
        drawJointDot(ctx, parts.leftThigh, { x: 0, y: UPPER_LEG_H / 2 }, jointSize);
        drawJointDot(ctx, parts.rightThigh, { x: 0, y: UPPER_LEG_H / 2 }, jointSize);
        drawJointDot(ctx, parts.leftCalf, { x: 0, y: LOWER_LEG_H / 2 }, jointSize);
        drawJointDot(ctx, parts.rightCalf, { x: 0, y: LOWER_LEG_H / 2 }, jointSize);

        ctx.restore();
    }

    function drawJointDot(ctx, body, offset, radius) {
        const cos = Math.cos(body.angle);
        const sin = Math.sin(body.angle);
        const wx = body.position.x + (offset.x * cos - offset.y * sin);
        const wy = body.position.y + (offset.x * sin + offset.y * cos);
        ctx.beginPath();
        ctx.arc(wx, wy, radius, 0, Math.PI * 2);
        ctx.fill();
    }

    return {
        create,
        addToWorld,
        removeFromWorld,
        applyControls,
        checkFallen,
        updateDistance,
        getCenter,
        render,
        TOTAL_HEIGHT: HEAD_RADIUS * 2 + TORSO_H + UPPER_LEG_H + LOWER_LEG_H + FOOT_H,
    };
})();
