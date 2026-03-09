/**
 * JREK Runner - Ragdoll physics body for Scott Jurek
 * Built with Matter.js rigid bodies and constraints
 *
 * Physics: QWOP-style paired joint motors.
 * - Body is RIGID at rest (joints locked, no collapse)
 * - J/R drive hips only (knees stay locked)
 * - E/K drive knees only (hips stay locked)
 * - Ankles always locked
 * - Split stance start: legs apart, knees slightly bent, feet on ground
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

    // === SPLIT STANCE GEOMETRY ===
    const HIP_SPLIT = 0.35;    // ~20 deg each leg from vertical
    const KNEE_BEND = 0.12;    // Slight bend at each knee

    // Height from torso center to bottom of front foot (for spawn positioning)
    const STANCE_HEIGHT = Math.round(
        TORSO_H / 2
        + Math.cos(HIP_SPLIT) * UPPER_LEG_H
        + Math.cos(HIP_SPLIT - KNEE_BEND) * LOWER_LEG_H
        + FOOT_H
    );

    // === MASS DISTRIBUTION ===
    const TORSO_DENSITY = 0.006;
    const LIMB_DENSITY = 0.001;
    const FOOT_DENSITY = 0.004;
    const HEAD_DENSITY = 0.0005;

    // === SURFACE FRICTION ===
    const BODY_FRICTION = 0.6;
    const BODY_RESTITUTION = 0.01;
    const FOOT_FRICTION = 3.0;

    // === JOINT MOTOR PARAMETERS ===
    const HIP_MOTOR_SPEED = 0.12;
    const KNEE_MOTOR_SPEED = 0.12;
    const HIP_MAX_TORQUE = 2.5;
    const KNEE_MAX_TORQUE = 2.0;
    const BRAKE_TORQUE = 3.0;
    const MOTOR_GAIN = 8.0;

    // === JOINT LOCKING ===
    // Kills relative angular velocity between two connected bodies.
    // At LOCK_RATE=0.95, joints lose 95% of relative rotation each frame.
    // This makes them effectively rigid without fighting the physics engine.
    const LOCK_RATE = 0.95;

    // === JOINT ANGLE LIMITS ===
    const HIP_MIN_ANGLE = -1.0;
    const HIP_MAX_ANGLE = 1.0;
    const KNEE_MIN_ANGLE = -1.8;
    const KNEE_MAX_ANGLE = 0.05;
    const ANKLE_MIN_ANGLE = -0.2;
    const ANKLE_MAX_ANGLE = 0.2;

    // === DAMPING ===
    const ANGULAR_DAMPING = 0.96;
    const MAX_ANGULAR_VEL = 0.3;

    // Constraint stiffness
    const JOINT_STIFFNESS = 1.0;
    const JOINT_DAMPING = 0.5;

    // === TORSO STABILIZATION ===
    // Simple PD to keep torso upright. Weak enough that movement is
    // precarious, strong enough that idle stance holds for several seconds.
    const TORSO_STAB_IDLE = 0.30;
    const TORSO_STAB_ACTIVE = 0.04;
    const TORSO_DAMP_IDLE = 0.85;
    const TORSO_DAMP_ACTIVE = 0.95;

    // === ROLLING CARTWHEEL EASTER EGG ===
    const ROLL_TORQUE = 0.6;
    const ROLL_FORWARD_FORCE = 0.001;

    // Collision categories
    const RUNNER_CATEGORY = 0x0002;
    const GROUND_CATEGORY = 0x0001;

    // --- Joint lock: kill relative angular velocity ---
    function lockJoint(parent, child) {
        const relAngVel = child.angularVelocity - parent.angularVelocity;
        Body.setAngularVelocity(child, child.angularVelocity - relAngVel * LOCK_RATE);
    }

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
            frictionAir: 0.01,
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
            frictionAir: 0.015,
        });

        // === SPLIT STANCE BODY POSITIONING ===
        // Each body part is placed along its kinematic chain using trig.
        // Left leg goes forward, right leg goes backward.

        const leftHipX = x - 3;
        const leftHipY = y + TORSO_H / 2;
        const rightHipX = x + 3;
        const rightHipY = y + TORSO_H / 2;

        // LEFT THIGH: angled forward by HIP_SPLIT
        const ltAngle = HIP_SPLIT;
        const ltCenterX = leftHipX + Math.sin(ltAngle) * UPPER_LEG_H / 2;
        const ltCenterY = leftHipY + Math.cos(ltAngle) * UPPER_LEG_H / 2;
        parts.leftThigh = Bodies.rectangle(ltCenterX, ltCenterY, UPPER_LEG_W, UPPER_LEG_H, {
            ...bodyOptions, density: LIMB_DENSITY, label: 'leftThigh', angle: ltAngle
        });

        // RIGHT THIGH: angled backward by -HIP_SPLIT
        const rtAngle = -HIP_SPLIT;
        const rtCenterX = rightHipX + Math.sin(rtAngle) * UPPER_LEG_H / 2;
        const rtCenterY = rightHipY + Math.cos(rtAngle) * UPPER_LEG_H / 2;
        parts.rightThigh = Bodies.rectangle(rtCenterX, rtCenterY, UPPER_LEG_W, UPPER_LEG_H, {
            ...bodyOptions, density: LIMB_DENSITY, label: 'rightThigh', angle: rtAngle
        });

        // Knee positions (bottom of thighs)
        const leftKneeX = leftHipX + Math.sin(ltAngle) * UPPER_LEG_H;
        const leftKneeY = leftHipY + Math.cos(ltAngle) * UPPER_LEG_H;
        const rightKneeX = rightHipX + Math.sin(rtAngle) * UPPER_LEG_H;
        const rightKneeY = rightHipY + Math.cos(rtAngle) * UPPER_LEG_H;

        // LEFT CALF: thigh angle minus knee bend (bends back relative to thigh)
        const lcAngle = ltAngle - KNEE_BEND;
        const lcCenterX = leftKneeX + Math.sin(lcAngle) * LOWER_LEG_H / 2;
        const lcCenterY = leftKneeY + Math.cos(lcAngle) * LOWER_LEG_H / 2;
        parts.leftCalf = Bodies.rectangle(lcCenterX, lcCenterY, LOWER_LEG_W, LOWER_LEG_H, {
            ...bodyOptions, density: LIMB_DENSITY, label: 'leftCalf', angle: lcAngle
        });

        // RIGHT CALF: thigh angle minus knee bend
        const rcAngle = rtAngle - KNEE_BEND;
        const rcCenterX = rightKneeX + Math.sin(rcAngle) * LOWER_LEG_H / 2;
        const rcCenterY = rightKneeY + Math.cos(rcAngle) * LOWER_LEG_H / 2;
        parts.rightCalf = Bodies.rectangle(rcCenterX, rcCenterY, LOWER_LEG_W, LOWER_LEG_H, {
            ...bodyOptions, density: LIMB_DENSITY, label: 'rightCalf', angle: rcAngle
        });

        // Ankle positions (bottom of calves)
        const leftAnkleX = leftKneeX + Math.sin(lcAngle) * LOWER_LEG_H;
        const leftAnkleY = leftKneeY + Math.cos(lcAngle) * LOWER_LEG_H;
        const rightAnkleX = rightKneeX + Math.sin(rcAngle) * LOWER_LEG_H;
        const rightAnkleY = rightKneeY + Math.cos(rcAngle) * LOWER_LEG_H;

        // LEFT FOOT: flat on ground (angle 0)
        parts.leftFoot = Bodies.rectangle(leftAnkleX, leftAnkleY + FOOT_H / 2, FOOT_W, FOOT_H, {
            ...bodyOptions, density: FOOT_DENSITY, friction: FOOT_FRICTION, label: 'leftFoot'
        });

        // RIGHT FOOT: flat on ground (angle 0)
        parts.rightFoot = Bodies.rectangle(rightAnkleX, rightAnkleY + FOOT_H / 2, FOOT_W, FOOT_H, {
            ...bodyOptions, density: FOOT_DENSITY, friction: FOOT_FRICTION, label: 'rightFoot'
        });

        // === CONSTRAINTS (pin joints) ===

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

        // Head stabilizer
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
            stiffness: 0.9,
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
            stiffness: 0.9,
            damping: 0.4,
            length: 0,
            label: 'rightAnkle',
        }));

        // === STRUCTURAL BRACES ===
        // Matter.js constraints are springs, not rigid joints.
        // These prevent positional collapse under gravity.

        // Full-leg braces: torso-hip to foot
        constraints.push(Constraint.create({
            bodyA: parts.torso,
            pointA: { x: -3, y: TORSO_H / 2 },
            bodyB: parts.leftFoot,
            pointB: { x: 0, y: 0 },
            stiffness: 0.2,
            damping: 0.3,
            length: UPPER_LEG_H + LOWER_LEG_H + FOOT_H / 2,
            label: 'leftFullLeg',
        }));

        constraints.push(Constraint.create({
            bodyA: parts.torso,
            pointA: { x: 3, y: TORSO_H / 2 },
            bodyB: parts.rightFoot,
            pointB: { x: 0, y: 0 },
            stiffness: 0.2,
            damping: 0.3,
            length: UPPER_LEG_H + LOWER_LEG_H + FOOT_H / 2,
            label: 'rightFullLeg',
        }));

        // Thigh cross-brace: prevents doing the splits
        constraints.push(Constraint.create({
            bodyA: parts.leftThigh,
            pointA: { x: 0, y: UPPER_LEG_H / 3 },
            bodyB: parts.rightThigh,
            pointB: { x: 0, y: UPPER_LEG_H / 3 },
            stiffness: 0.12,
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
            rolling: false,
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

    // === JOINT MOTOR (TORQUE-BASED) ===
    function applyJointMotor(parent, child, targetSpeed, maxTorque) {
        const relAngVel = child.angularVelocity - parent.angularVelocity;
        let torque = (targetSpeed - relAngVel) * MOTOR_GAIN;
        if (torque > maxTorque) torque = maxTorque;
        if (torque < -maxTorque) torque = -maxTorque;
        child.torque += torque;
        parent.torque -= torque;
    }

    // === ANGLE LIMIT ENFORCEMENT ===
    function enforceAngleLimit(parent, child, minAngle, maxAngle) {
        let relAngle = child.angle - parent.angle;
        while (relAngle > Math.PI) relAngle -= 2 * Math.PI;
        while (relAngle < -Math.PI) relAngle += 2 * Math.PI;

        if (relAngle < minAngle) {
            const overshoot = minAngle - relAngle;
            Body.setAngularVelocity(child, child.angularVelocity + overshoot * 0.5);
            Body.setAngularVelocity(parent, parent.angularVelocity - overshoot * 0.15);
        }
        if (relAngle > maxAngle) {
            const overshoot = relAngle - maxAngle;
            Body.setAngularVelocity(child, child.angularVelocity - overshoot * 0.5);
            Body.setAngularVelocity(parent, parent.angularVelocity + overshoot * 0.15);
        }
    }

    /**
     * Core control function -- called every physics frame.
     *
     * SIMPLIFIED CONTROLS:
     * - J/R = hips only (knees LOCKED, don't bend)
     * - E/K = knees only (hips LOCKED, don't move)
     * - No keys = ALL joints locked rigid
     * - All four = rolling easter egg
     */
    function applyControls(runner, keys) {
        const { torso, leftThigh, rightThigh, leftCalf, rightCalf, leftFoot, rightFoot } = runner.parts;

        const allPressed = keys.j && keys.r && keys.e && keys.k;
        const anyPressed = keys.j || keys.r || keys.e || keys.k;
        const anyHipKey = keys.j || keys.r;
        const anyKneeKey = keys.e || keys.k;
        runner.rolling = allPressed;

        // --- Angular damping + velocity clamping on all bodies ---
        for (const key of Object.keys(runner.parts)) {
            const part = runner.parts[key];
            const damping = (allPressed && part === torso) ? 0.99 : ANGULAR_DAMPING;
            let av = part.angularVelocity * damping;
            if (!allPressed) {
                av = Math.max(-MAX_ANGULAR_VEL, Math.min(MAX_ANGULAR_VEL, av));
            }
            Body.setAngularVelocity(part, av);
        }

        // --- Torso stabilization (PD controller) ---
        // Keeps the torso upright. Weaker when keys are pressed so
        // movement is precarious and falling is possible.
        if (!allPressed) {
            const stabStrength = anyPressed ? TORSO_STAB_ACTIVE : TORSO_STAB_IDLE;
            const dampFactor = anyPressed ? TORSO_DAMP_ACTIVE : TORSO_DAMP_IDLE;
            Body.setAngularVelocity(torso,
                torso.angularVelocity * dampFactor - torso.angle * stabStrength
            );
        }

        // === Easter egg: all four keys = rolling cartwheel ===
        if (allPressed) {
            torso.torque += ROLL_TORQUE;
            Body.applyForce(torso, torso.position, {
                x: ROLL_FORWARD_FORCE,
                y: 0,
            });

            const phase = Math.sin(torso.angle * 2);
            applyJointMotor(torso, leftThigh, phase > 0 ? HIP_MOTOR_SPEED : -HIP_MOTOR_SPEED, HIP_MAX_TORQUE);
            applyJointMotor(torso, rightThigh, phase > 0 ? -HIP_MOTOR_SPEED : HIP_MOTOR_SPEED, HIP_MAX_TORQUE);
            applyJointMotor(leftThigh, leftCalf, KNEE_MOTOR_SPEED * 0.4, KNEE_MAX_TORQUE);
            applyJointMotor(rightThigh, rightCalf, KNEE_MOTOR_SPEED * 0.4, KNEE_MAX_TORQUE);
            applyJointMotor(leftCalf, leftFoot, 0, BRAKE_TORQUE);
            applyJointMotor(rightCalf, rightFoot, 0, BRAKE_TORQUE);

            enforceAngleLimit(torso, leftThigh, -2.2, 2.2);
            enforceAngleLimit(torso, rightThigh, -2.2, 2.2);
            enforceAngleLimit(leftThigh, leftCalf, -2.5, 0.5);
            enforceAngleLimit(rightThigh, rightCalf, -2.5, 0.5);
            enforceAngleLimit(leftCalf, leftFoot, -0.6, 0.6);
            enforceAngleLimit(rightCalf, rightFoot, -0.6, 0.6);

            return;
        }

        // --- Determine motor targets ---
        let leftHipTarget = 0;
        let rightHipTarget = 0;
        let leftKneeTarget = 0;
        let rightKneeTarget = 0;

        if (keys.j && !keys.r) {
            leftHipTarget = -HIP_MOTOR_SPEED;
            rightHipTarget = HIP_MOTOR_SPEED;
        } else if (keys.r && !keys.j) {
            leftHipTarget = HIP_MOTOR_SPEED;
            rightHipTarget = -HIP_MOTOR_SPEED;
        }

        if (keys.e && !keys.k) {
            leftKneeTarget = KNEE_MOTOR_SPEED;
            rightKneeTarget = -KNEE_MOTOR_SPEED;
        } else if (keys.k && !keys.e) {
            leftKneeTarget = -KNEE_MOTOR_SPEED;
            rightKneeTarget = KNEE_MOTOR_SPEED;
        }

        // --- Apply motors for active joints, LOCK inactive joints ---

        if (anyHipKey) {
            // Drive hips
            applyJointMotor(torso, leftThigh, leftHipTarget, HIP_MAX_TORQUE);
            applyJointMotor(torso, rightThigh, rightHipTarget, HIP_MAX_TORQUE);
        } else {
            // Lock hips rigid
            lockJoint(torso, leftThigh);
            lockJoint(torso, rightThigh);
        }

        if (anyKneeKey) {
            // Drive knees
            applyJointMotor(leftThigh, leftCalf, leftKneeTarget, KNEE_MAX_TORQUE);
            applyJointMotor(rightThigh, rightCalf, rightKneeTarget, KNEE_MAX_TORQUE);
        } else {
            // Lock knees rigid
            lockJoint(leftThigh, leftCalf);
            lockJoint(rightThigh, rightCalf);
        }

        // Cross-locking: when hips are active, lock knees too (and vice versa)
        if (anyHipKey && !anyKneeKey) {
            lockJoint(leftThigh, leftCalf);
            lockJoint(rightThigh, rightCalf);
        }
        if (anyKneeKey && !anyHipKey) {
            lockJoint(torso, leftThigh);
            lockJoint(torso, rightThigh);
        }

        // Ankles: ALWAYS locked rigid
        lockJoint(leftCalf, leftFoot);
        lockJoint(rightCalf, rightFoot);

        // --- Enforce angle limits ---
        enforceAngleLimit(torso, leftThigh, HIP_MIN_ANGLE, HIP_MAX_ANGLE);
        enforceAngleLimit(torso, rightThigh, HIP_MIN_ANGLE, HIP_MAX_ANGLE);
        enforceAngleLimit(leftThigh, leftCalf, KNEE_MIN_ANGLE, KNEE_MAX_ANGLE);
        enforceAngleLimit(rightThigh, rightCalf, KNEE_MIN_ANGLE, KNEE_MAX_ANGLE);
        enforceAngleLimit(leftCalf, leftFoot, ANKLE_MIN_ANGLE, ANKLE_MAX_ANGLE);
        enforceAngleLimit(rightCalf, rightFoot, ANKLE_MIN_ANGLE, ANKLE_MAX_ANGLE);
    }

    // Check if the runner has fallen
    function checkFallen(runner, groundY) {
        if (runner.rolling) return false;

        const head = runner.parts.head;
        const torso = runner.parts.torso;

        const torsoAngle = Math.abs(torso.angle);
        const headLow = head.position.y > groundY - 15;
        const torsoTilted = torsoAngle > 1.4;

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
    function render(ctx, runner, cameraX, cameraY) {
        const parts = runner.parts;
        const ox = -cameraX;
        const oy = -cameraY;

        ctx.save();
        ctx.translate(ox, oy);

        // Color palette - Jurek at Western States circa 2003-2005
        const skinColor = '#c49a6c';
        const skinShadow = '#a8825a';
        const cropTopColor = '#f0ebe3';
        const cropTopShadow = '#d8d0c4';
        const shortsColor = '#1a1a1a';
        const gearBeltColor = '#333333';
        const shoeColor = '#4a6741';
        const shoeSoleColor = '#1a1a1a';
        const capRedColor = '#cc2222';
        const capWhiteColor = '#e8e0d4';
        const capBrimColor = '#2a2a2a';
        const hairColor = '#3b2507';
        const wristbandColor = '#e6cc00';
        const watchColor = '#222222';
        const bottleColor = '#e8e8e8';

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

        function drawShoe(foot, isBack) {
            ctx.save();
            ctx.translate(foot.position.x, foot.position.y);
            ctx.rotate(foot.angle);
            ctx.fillStyle = shoeColor;
            ctx.beginPath();
            ctx.roundRect(-FOOT_W / 2, -FOOT_H / 2, FOOT_W, FOOT_H, 3);
            ctx.fill();
            ctx.fillStyle = shoeSoleColor;
            ctx.fillRect(-FOOT_W / 2, FOOT_H / 2 - 2, FOOT_W, 2);
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

        ctx.fillStyle = skinColor;
        ctx.beginPath();
        ctx.roundRect(-TORSO_W / 2, -TORSO_H / 2, TORSO_W, TORSO_H, 4);
        ctx.fill();

        const cropHeight = TORSO_H * 0.50;
        ctx.fillStyle = cropTopColor;
        ctx.beginPath();
        ctx.roundRect(-TORSO_W / 2 - 1, -TORSO_H / 2, TORSO_W + 2, cropHeight, [4, 4, 0, 0]);
        ctx.fill();

        ctx.strokeStyle = cropTopShadow;
        ctx.lineWidth = 0.6;
        ctx.beginPath();
        ctx.moveTo(-TORSO_W / 4, -TORSO_H / 2 + cropHeight * 0.3);
        ctx.lineTo(TORSO_W / 4, -TORSO_H / 2 + cropHeight * 0.35);
        ctx.moveTo(-TORSO_W / 3, -TORSO_H / 2 + cropHeight * 0.6);
        ctx.lineTo(TORSO_W / 5, -TORSO_H / 2 + cropHeight * 0.55);
        ctx.stroke();

        ctx.strokeStyle = '#c0b8aa';
        ctx.lineWidth = 1.2;
        ctx.beginPath();
        ctx.moveTo(-TORSO_W / 2 - 1, -TORSO_H / 2 + cropHeight);
        ctx.lineTo(TORSO_W / 2 + 1, -TORSO_H / 2 + cropHeight + 1);
        ctx.stroke();

        const shortsTop = TORSO_H / 4;
        const shortsLength = TORSO_H / 2 - shortsTop + 8;
        ctx.fillStyle = shortsColor;
        ctx.beginPath();
        ctx.roundRect(-TORSO_W / 2 - 2, shortsTop, TORSO_W + 4, shortsLength, [0, 0, 3, 3]);
        ctx.fill();

        ctx.fillStyle = gearBeltColor;
        ctx.fillRect(-TORSO_W / 2 - 2, shortsTop - 1, TORSO_W + 4, 4);
        ctx.fillStyle = '#444444';
        ctx.fillRect(TORSO_W / 2 - 2, shortsTop - 2, 5, 6);

        ctx.restore();

        // -- FRONT LEG (right, brighter) --
        drawBodyPart(parts.rightThigh, UPPER_LEG_W, UPPER_LEG_H, skinColor);
        drawBodyPart(parts.rightCalf, LOWER_LEG_W, LOWER_LEG_H, skinColor);
        drawShoe(parts.rightFoot, false);

        // -- HANDHELD BOTTLE --
        ctx.save();
        ctx.translate(parts.torso.position.x, parts.torso.position.y);
        ctx.rotate(parts.torso.angle);
        ctx.fillStyle = bottleColor;
        ctx.beginPath();
        ctx.roundRect(TORSO_W / 2 - 2, -2, 5, 10, 2);
        ctx.fill();
        ctx.fillStyle = '#aaaaaa';
        ctx.beginPath();
        ctx.arc(TORSO_W / 2 + 0.5, -2, 2.5, Math.PI, 0);
        ctx.fill();
        ctx.fillStyle = wristbandColor;
        ctx.fillRect(TORSO_W / 2 - 3, 8, 7, 3);
        ctx.fillStyle = watchColor;
        ctx.fillRect(-TORSO_W / 2 - 3, 6, 5, 4);
        ctx.fillStyle = '#335533';
        ctx.fillRect(-TORSO_W / 2 - 2, 7, 3, 2);
        ctx.restore();

        // -- HEAD --
        ctx.save();
        ctx.translate(parts.head.position.x, parts.head.position.y);
        ctx.rotate(parts.head.angle);

        ctx.fillStyle = hairColor;
        ctx.beginPath();
        ctx.arc(0, 0, HEAD_RADIUS + 2, Math.PI * 0.6, Math.PI * 1.4);
        ctx.quadraticCurveTo(-HEAD_RADIUS - 4, HEAD_RADIUS + 6, -HEAD_RADIUS - 2, HEAD_RADIUS + 12);
        ctx.quadraticCurveTo(-HEAD_RADIUS + 2, HEAD_RADIUS + 10, -HEAD_RADIUS + 4, HEAD_RADIUS + 4);
        ctx.fill();

        ctx.fillStyle = skinColor;
        ctx.beginPath();
        ctx.arc(0, 0, HEAD_RADIUS, 0, Math.PI * 2);
        ctx.fill();

        ctx.fillStyle = capRedColor;
        ctx.beginPath();
        ctx.arc(0, -2, HEAD_RADIUS + 1, Math.PI * 1.15, Math.PI * 1.85);
        ctx.quadraticCurveTo(0, -HEAD_RADIUS - 6, 0, -HEAD_RADIUS - 4);
        ctx.closePath();
        ctx.fill();

        ctx.fillStyle = capWhiteColor;
        ctx.beginPath();
        ctx.arc(0, -2, HEAD_RADIUS + 1, Math.PI * 0.15, Math.PI * 0.35);
        ctx.lineTo(0, -HEAD_RADIUS - 4);
        ctx.arc(0, -2, HEAD_RADIUS + 1, Math.PI * 1.85, Math.PI * 2.0);
        ctx.closePath();
        ctx.fill();

        ctx.fillStyle = capBrimColor;
        ctx.beginPath();
        ctx.moveTo(HEAD_RADIUS * 0.5, -HEAD_RADIUS * 0.5);
        ctx.quadraticCurveTo(HEAD_RADIUS + 8, -HEAD_RADIUS * 0.6, HEAD_RADIUS + 11, -HEAD_RADIUS * 0.25);
        ctx.quadraticCurveTo(HEAD_RADIUS + 8, HEAD_RADIUS * 0.0, HEAD_RADIUS * 0.4, -HEAD_RADIUS * 0.15);
        ctx.closePath();
        ctx.fill();

        ctx.fillStyle = '#ffffff';
        ctx.font = 'bold 5px Arial';
        ctx.textAlign = 'center';
        ctx.fillText('CLIF', 0, -HEAD_RADIUS + 2);

        ctx.fillStyle = '#1a1208';
        ctx.beginPath();
        ctx.arc(-4, -1, 1.8, 0, Math.PI * 2);
        ctx.arc(4, -1, 1.8, 0, Math.PI * 2);
        ctx.fill();

        ctx.strokeStyle = '#2a1a08';
        ctx.lineWidth = 1.3;
        ctx.beginPath();
        ctx.moveTo(-6, -4);
        ctx.lineTo(-2, -3.5);
        ctx.moveTo(2, -3.5);
        ctx.lineTo(6, -4);
        ctx.stroke();

        ctx.strokeStyle = '#a07850';
        ctx.lineWidth = 0.8;
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(1, 2);
        ctx.stroke();

        ctx.strokeStyle = '#1a1208';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        if (runner.fallen) {
            ctx.arc(0, 5, 3.5, 0, Math.PI * 2);
            ctx.fillStyle = '#1a1208';
            ctx.fill();
        } else if (runner.rolling) {
            ctx.moveTo(-5, 4);
            ctx.quadraticCurveTo(0, 9, 5, 4);
        } else {
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
        STANCE_HEIGHT,
    };
})();
