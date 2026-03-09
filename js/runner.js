/**
 * JREK Runner - Ragdoll physics body for Scott Jurek
 * Built with Matter.js rigid bodies and constraints
 */

const Runner = (function() {
    const { Bodies, Body, Composite, Constraint, Vector } = Matter;

    // Body part dimensions (pixels) - tuned for playability
    const SCALE = 1.0;
    const HEAD_RADIUS = 12 * SCALE;
    const TORSO_W = 16 * SCALE;
    const TORSO_H = 40 * SCALE;
    const UPPER_LEG_W = 10 * SCALE;
    const UPPER_LEG_H = 32 * SCALE;
    const LOWER_LEG_W = 8 * SCALE;
    const LOWER_LEG_H = 30 * SCALE;
    const FOOT_W = 20 * SCALE;
    const FOOT_H = 6 * SCALE;

    // Physics tuning
    const BODY_FRICTION = 0.6;
    const BODY_RESTITUTION = 0.05;
    const LIMB_DENSITY = 0.001;
    const TORSO_DENSITY = 0.005;
    const HEAD_DENSITY = 0.002;
    const FOOT_DENSITY = 0.004;

    // Joint stiffness
    const JOINT_STIFFNESS = 1.0;
    const JOINT_DAMPING = 0.5;

    // Force magnitudes for controls
    const HIP_TORQUE = 0.08;
    const KNEE_TORQUE = 0.06;

    // Standing stabilization - angular springs that keep the runner upright
    const TORSO_UPRIGHT_FORCE = 0.025;
    const HIP_SPRING_FORCE = 0.015;
    const KNEE_SPRING_FORCE = 0.012;
    const ANGULAR_DAMPING = 0.15;

    // Collision categories
    const RUNNER_CATEGORY = 0x0002;
    const GROUND_CATEGORY = 0x0001;

    function create(x, y) {
        const parts = {};
        const constraints = [];

        const collisionFilter = {
            category: RUNNER_CATEGORY,
            mask: GROUND_CATEGORY, // Collide with ground only, not self
        };

        const bodyOptions = {
            friction: BODY_FRICTION,
            restitution: BODY_RESTITUTION,
            collisionFilter: collisionFilter,
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
        });

        // LEFT UPPER LEG (thigh)
        parts.leftThigh = Bodies.rectangle(
            x - 4, y + TORSO_H / 2 + UPPER_LEG_H / 2,
            UPPER_LEG_W, UPPER_LEG_H,
            {
                ...bodyOptions,
                density: LIMB_DENSITY,
                label: 'leftThigh',
            }
        );

        // RIGHT UPPER LEG (thigh)
        parts.rightThigh = Bodies.rectangle(
            x + 4, y + TORSO_H / 2 + UPPER_LEG_H / 2,
            UPPER_LEG_W, UPPER_LEG_H,
            {
                ...bodyOptions,
                density: LIMB_DENSITY,
                label: 'rightThigh',
            }
        );

        // LEFT LOWER LEG (calf)
        parts.leftCalf = Bodies.rectangle(
            x - 4, y + TORSO_H / 2 + UPPER_LEG_H + LOWER_LEG_H / 2,
            LOWER_LEG_W, LOWER_LEG_H,
            {
                ...bodyOptions,
                density: LIMB_DENSITY,
                label: 'leftCalf',
            }
        );

        // RIGHT LOWER LEG (calf)
        parts.rightCalf = Bodies.rectangle(
            x + 4, y + TORSO_H / 2 + UPPER_LEG_H + LOWER_LEG_H / 2,
            LOWER_LEG_W, LOWER_LEG_H,
            {
                ...bodyOptions,
                density: LIMB_DENSITY,
                label: 'rightCalf',
            }
        );

        // LEFT FOOT
        parts.leftFoot = Bodies.rectangle(
            x - 4, y + TORSO_H / 2 + UPPER_LEG_H + LOWER_LEG_H + FOOT_H / 2,
            FOOT_W, FOOT_H,
            {
                ...bodyOptions,
                density: FOOT_DENSITY,
                friction: 0.9,
                label: 'leftFoot',
            }
        );

        // RIGHT FOOT
        parts.rightFoot = Bodies.rectangle(
            x + 4, y + TORSO_H / 2 + UPPER_LEG_H + LOWER_LEG_H + FOOT_H / 2,
            FOOT_W, FOOT_H,
            {
                ...bodyOptions,
                density: FOOT_DENSITY,
                friction: 0.9,
                label: 'rightFoot',
            }
        );

        // CONSTRAINTS (joints)

        // Neck: head to torso
        constraints.push(Constraint.create({
            bodyA: parts.head,
            pointA: { x: 0, y: HEAD_RADIUS * 0.8 },
            bodyB: parts.torso,
            pointB: { x: 0, y: -TORSO_H / 2 },
            stiffness: 0.8,
            damping: 0.4,
            length: 2,
            label: 'neck',
        }));

        // Left Hip: torso to left thigh
        constraints.push(Constraint.create({
            bodyA: parts.torso,
            pointA: { x: -4, y: TORSO_H / 2 },
            bodyB: parts.leftThigh,
            pointB: { x: 0, y: -UPPER_LEG_H / 2 },
            stiffness: JOINT_STIFFNESS,
            damping: JOINT_DAMPING,
            length: 0,
            label: 'leftHip',
        }));

        // Right Hip: torso to right thigh
        constraints.push(Constraint.create({
            bodyA: parts.torso,
            pointA: { x: 4, y: TORSO_H / 2 },
            bodyB: parts.rightThigh,
            pointB: { x: 0, y: -UPPER_LEG_H / 2 },
            stiffness: JOINT_STIFFNESS,
            damping: JOINT_DAMPING,
            length: 0,
            label: 'rightHip',
        }));

        // Left Knee: left thigh to left calf
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

        // Right Knee: right thigh to right calf
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

        // Left Ankle: left calf to left foot
        constraints.push(Constraint.create({
            bodyA: parts.leftCalf,
            pointA: { x: 0, y: LOWER_LEG_H / 2 },
            bodyB: parts.leftFoot,
            pointB: { x: -2, y: 0 },
            stiffness: 0.7,
            damping: 0.3,
            length: 0,
            label: 'leftAnkle',
        }));

        // Right Ankle: right calf to right foot
        constraints.push(Constraint.create({
            bodyA: parts.rightCalf,
            pointA: { x: 0, y: LOWER_LEG_H / 2 },
            bodyB: parts.rightFoot,
            pointB: { x: -2, y: 0 },
            stiffness: 0.7,
            damping: 0.3,
            length: 0,
            label: 'rightAnkle',
        }));

        // Angular limit constraints (prevent hyper-extension)
        // These extra constraints act as "muscles" to limit joint range

        // Torso stabilizer - keeps torso somewhat upright
        constraints.push(Constraint.create({
            bodyA: parts.head,
            pointA: { x: 0, y: 0 },
            bodyB: parts.torso,
            pointB: { x: 0, y: -TORSO_H / 2 + 5 },
            stiffness: 0.15,
            damping: 0.2,
            length: HEAD_RADIUS + 5,
            label: 'headStabilizer',
        }));

        // Cross-brace: left thigh to right thigh (prevents splits)
        constraints.push(Constraint.create({
            bodyA: parts.leftThigh,
            pointA: { x: 0, y: UPPER_LEG_H / 4 },
            bodyB: parts.rightThigh,
            pointB: { x: 0, y: UPPER_LEG_H / 4 },
            stiffness: 0.1,
            damping: 0.2,
            length: 12,
            label: 'thighBrace',
        }));

        // Additional structural constraints for standing stability

        // Torso to left knee region (prevents legs folding under)
        constraints.push(Constraint.create({
            bodyA: parts.torso,
            pointA: { x: -4, y: TORSO_H / 2 },
            bodyB: parts.leftCalf,
            pointB: { x: 0, y: -LOWER_LEG_H / 4 },
            stiffness: 0.05,
            damping: 0.2,
            length: UPPER_LEG_H + LOWER_LEG_H / 4,
            label: 'leftLegBrace',
        }));

        // Torso to right knee region
        constraints.push(Constraint.create({
            bodyA: parts.torso,
            pointA: { x: 4, y: TORSO_H / 2 },
            bodyB: parts.rightCalf,
            pointB: { x: 0, y: -LOWER_LEG_H / 4 },
            stiffness: 0.05,
            damping: 0.2,
            length: UPPER_LEG_H + LOWER_LEG_H / 4,
            label: 'rightLegBrace',
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

    // Stabilize the runner - angular springs keep body upright when no keys pressed.
    // This is the key to making the runner STAND like in QWOP.
    // Without this, Matter.js ragdolls just collapse.
    function stabilize(runner) {
        const { torso, leftThigh, rightThigh, leftCalf, rightCalf, leftFoot, rightFoot } = runner.parts;

        // Torso wants to stay vertical (angle = 0)
        const torsoCorrection = -torso.angle * TORSO_UPRIGHT_FORCE;
        Body.setAngularVelocity(torso,
            torso.angularVelocity * (1 - ANGULAR_DAMPING) + torsoCorrection
        );

        // Thighs want to hang straight down relative to torso (match torso angle)
        const leftThighTarget = torso.angle; // straight down = same as torso
        const rightThighTarget = torso.angle;
        const leftThighError = leftThigh.angle - leftThighTarget;
        const rightThighError = rightThigh.angle - rightThighTarget;

        Body.setAngularVelocity(leftThigh,
            leftThigh.angularVelocity * (1 - ANGULAR_DAMPING) - leftThighError * HIP_SPRING_FORCE
        );
        Body.setAngularVelocity(rightThigh,
            rightThigh.angularVelocity * (1 - ANGULAR_DAMPING) - rightThighError * HIP_SPRING_FORCE
        );

        // Calves want to stay straight relative to their thigh (knee straight)
        const leftKneeError = leftCalf.angle - leftThigh.angle;
        const rightKneeError = rightCalf.angle - rightThigh.angle;

        Body.setAngularVelocity(leftCalf,
            leftCalf.angularVelocity * (1 - ANGULAR_DAMPING) - leftKneeError * KNEE_SPRING_FORCE
        );
        Body.setAngularVelocity(rightCalf,
            rightCalf.angularVelocity * (1 - ANGULAR_DAMPING) - rightKneeError * KNEE_SPRING_FORCE
        );

        // Feet want to stay flat
        Body.setAngularVelocity(leftFoot,
            leftFoot.angularVelocity * (1 - ANGULAR_DAMPING * 2) - leftFoot.angle * 0.01
        );
        Body.setAngularVelocity(rightFoot,
            rightFoot.angularVelocity * (1 - ANGULAR_DAMPING * 2) - rightFoot.angle * 0.01
        );
    }

    // Apply forces based on key states
    // Controls OVERRIDE the stabilization, creating movement (and chaos).
    // J/R swing thighs, E/K extend calves. Alternating creates a gait.
    function applyControls(runner, keys) {
        const { torso, leftThigh, rightThigh, leftCalf, rightCalf } = runner.parts;

        // Always run stabilization first (keeps runner standing)
        stabilize(runner);

        const anyKeyPressed = keys.j || keys.r || keys.e || keys.k;

        // When keys are pressed, reduce stabilization influence
        // by applying stronger override forces
        const forceMultiplier = anyKeyPressed ? 1.0 : 0.0;

        // J key: left thigh swings forward (clockwise), right thigh pulls back
        if (keys.j) {
            Body.setAngularVelocity(leftThigh,
                leftThigh.angularVelocity + HIP_TORQUE
            );
            Body.setAngularVelocity(rightThigh,
                rightThigh.angularVelocity - HIP_TORQUE * 0.5
            );
            Body.setAngularVelocity(torso,
                torso.angularVelocity - HIP_TORQUE * 0.15
            );
        }

        // R key: right thigh swings forward, left pulls back
        if (keys.r) {
            Body.setAngularVelocity(rightThigh,
                rightThigh.angularVelocity + HIP_TORQUE
            );
            Body.setAngularVelocity(leftThigh,
                leftThigh.angularVelocity - HIP_TORQUE * 0.5
            );
            Body.setAngularVelocity(torso,
                torso.angularVelocity - HIP_TORQUE * 0.15
            );
        }

        // E key: left calf extends (straightens knee)
        if (keys.e) {
            Body.setAngularVelocity(leftCalf,
                leftCalf.angularVelocity - KNEE_TORQUE
            );
            Body.setAngularVelocity(leftThigh,
                leftThigh.angularVelocity + KNEE_TORQUE * 0.3
            );
        }

        // K key: right calf extends
        if (keys.k) {
            Body.setAngularVelocity(rightCalf,
                rightCalf.angularVelocity - KNEE_TORQUE
            );
            Body.setAngularVelocity(rightThigh,
                rightThigh.angularVelocity + KNEE_TORQUE * 0.3
            );
        }
    }

    // Check if the runner has fallen (head or torso touching ground level or torso very tilted)
    function checkFallen(runner, groundY) {
        const head = runner.parts.head;
        const torso = runner.parts.torso;

        // Head below a certain point, or torso rotated more than ~80 degrees
        const torsoAngle = Math.abs(torso.angle);
        const headLow = head.position.y > groundY - 20;
        const torsoTilted = torsoAngle > 1.4; // ~80 degrees

        if ((headLow || torsoTilted) && !runner.fallen) {
            runner.fallen = true;
            runner.fallTime = Date.now();
            return true;
        }
        return false;
    }

    // Calculate distance traveled in pixels from start
    function updateDistance(runner) {
        const torsoX = runner.parts.torso.position.x;
        runner.distance = Math.max(0, torsoX - runner.startX);
        runner.maxDistance = Math.max(runner.maxDistance, runner.distance);
        return runner.distance;
    }

    // Get the center position of the runner (for camera tracking)
    function getCenter(runner) {
        return {
            x: runner.parts.torso.position.x,
            y: runner.parts.torso.position.y,
        };
    }

    // Draw the runner on canvas
    function render(ctx, runner, cameraX, cameraY) {
        const parts = runner.parts;
        const ox = -cameraX;
        const oy = -cameraY;

        ctx.save();
        ctx.translate(ox, oy);

        // Color palette
        const skinColor = '#d4a574';
        const shirtColor = '#2d5a3d'; // Forest green singlet
        const shortsColor = '#1a1a2e';
        const shoeColor = '#4a3728';
        const bandanaColor = '#cc3333'; // Red bandana

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

        // Draw order: back leg, torso, front leg, head

        // Left leg (back)
        drawBodyPart(parts.leftThigh, UPPER_LEG_W, UPPER_LEG_H, skinColor);
        drawBodyPart(parts.leftCalf, LOWER_LEG_W, LOWER_LEG_H, skinColor);
        drawBodyPart(parts.leftFoot, FOOT_W, FOOT_H, shoeColor);

        // Right leg (front)
        drawBodyPart(parts.rightThigh, UPPER_LEG_W, UPPER_LEG_H, skinColor);
        drawBodyPart(parts.rightCalf, LOWER_LEG_W, LOWER_LEG_H, skinColor);
        drawBodyPart(parts.rightFoot, FOOT_W, FOOT_H, shoeColor);

        // Torso (singlet)
        drawBodyPart(parts.torso, TORSO_W, TORSO_H, shirtColor);

        // Shorts overlay on torso bottom
        ctx.save();
        ctx.translate(parts.torso.position.x, parts.torso.position.y);
        ctx.rotate(parts.torso.angle);
        ctx.fillStyle = shortsColor;
        ctx.fillRect(-TORSO_W / 2 - 1, TORSO_H / 4, TORSO_W + 2, TORSO_H / 4 + 2);
        ctx.restore();

        // Head
        ctx.save();
        ctx.translate(parts.head.position.x, parts.head.position.y);
        ctx.rotate(parts.head.angle);

        // Head circle (skin)
        ctx.fillStyle = skinColor;
        ctx.beginPath();
        ctx.arc(0, 0, HEAD_RADIUS, 0, Math.PI * 2);
        ctx.fill();

        // Bandana (Jurek's signature)
        ctx.fillStyle = bandanaColor;
        ctx.beginPath();
        ctx.arc(0, -1, HEAD_RADIUS + 1, Math.PI * 1.1, Math.PI * 1.9);
        ctx.lineTo(HEAD_RADIUS + 6, -4);
        ctx.lineTo(HEAD_RADIUS + 3, 2);
        ctx.closePath();
        ctx.fill();

        // Simple face
        ctx.fillStyle = '#2a1f14';
        // Eyes
        ctx.beginPath();
        ctx.arc(-4, -2, 1.5, 0, Math.PI * 2);
        ctx.arc(4, -2, 1.5, 0, Math.PI * 2);
        ctx.fill();

        // Determined mouth (or grimace if fallen)
        ctx.strokeStyle = '#2a1f14';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        if (runner.fallen) {
            // Open mouth O face
            ctx.arc(0, 4, 3, 0, Math.PI * 2);
        } else {
            // Determined line
            ctx.moveTo(-3, 4);
            ctx.lineTo(3, 4);
        }
        ctx.stroke();

        ctx.restore();

        // Joint dots (small circles at connection points for visual clarity)
        ctx.fillStyle = 'rgba(0,0,0,0.3)';
        const jointSize = 3;

        // Hip joints
        drawJointDot(ctx, parts.torso, { x: -4, y: TORSO_H / 2 }, jointSize);
        drawJointDot(ctx, parts.torso, { x: 4, y: TORSO_H / 2 }, jointSize);

        // Knee joints
        drawJointDot(ctx, parts.leftThigh, { x: 0, y: UPPER_LEG_H / 2 }, jointSize);
        drawJointDot(ctx, parts.rightThigh, { x: 0, y: UPPER_LEG_H / 2 }, jointSize);

        // Ankle joints
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
        // Expose dimensions for terrain alignment
        TOTAL_HEIGHT: HEAD_RADIUS * 2 + TORSO_H + UPPER_LEG_H + LOWER_LEG_H + FOOT_H,
    };
})();
