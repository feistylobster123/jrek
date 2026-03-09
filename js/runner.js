/**
 * JREK Runner - Ragdoll physics body for Scott Jurek
 * Built with Matter.js rigid bodies and constraints
 *
 * v0.8 Physics: Nick's simplified rigid-body approach.
 * - Body is a RIGID STRUCTURE when no buttons pressed.
 *   Every joint has a PD controller holding it at its rest angle.
 *   No separate torso stabilization -- rigidity comes from ALL joints
 *   acting as one unit. The whole body is an inverted pendulum.
 * - J/R drive hips only (knees + ankles stay locked)
 * - E/K drive knees only (hips + ankles stay locked)
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
    // Motors now drive toward TARGET ANGLES (not speeds).
    // When J/R/E/K is pressed, the joint's target angle shifts from its rest position.
    // Uses the same PD controller as lockJoint but with a softer response.
    // This creates real constraint forces: if the foot is planted, the body moves.
    const HIP_DRIVE_ANGLE = 0.40;    // How far hips swing from rest (radians, ~23deg)
    const KNEE_DRIVE_ANGLE = 0.35;   // How far knees bend from rest
    const MOTOR_POS_GAIN = 0.80;     // Same strength as lock PD (motor = lock at shifted target)
    const MOTOR_DAMP = 0.95;         // Strong damping to prevent oscillation

    // === PASSIVE JOINT MODE ===
    // When a joint's neighbors are actively driven, this joint needs to be SOFT
    // so the limb can flex. Full-strength locks make the leg a rigid rod,
    // preventing ground reaction forces. Passive mode = gentle guidance.
    const PASSIVE_POS_GAIN = 0.10;   // Much softer than full lock (0.80)
    const PASSIVE_DAMP = 0.60;       // Allow significant angular velocity

    // === JOINT LOCKING (VELOCITY-BASED PD PER JOINT) ===
    // Each joint uses a PD controller to hold its rest angle.
    // Handles RELATIVE angles between connected bodies.
    // Must be strong enough to overpower the pin constraint's tendency
    // to collapse joints toward 0 relative angle (constraint restoring torque).
    const LOCK_DAMP = 0.98;     // Kill relative angular velocity per frame
    const LOCK_POS_GAIN = 0.80; // Position correction: velocity += error * gain

    // === TORSO BALANCE (VELOCITY-BASED PD) ===
    // Handles ABSOLUTE upright balance (the inverted pendulum problem).
    // Per-joint PD can't detect whole-body tilt because hip/knee relative
    // angles stay constant when the body tilts as a unit. Only the torso PD
    // references the world frame (angle=0 = upright).
    const TORSO_STAB_IDLE = 0.30;   // P gain: velocity correction per radian of tilt
    const TORSO_STAB_ACTIVE = 0.04; // Much weaker when keys pressed (allows lean for ground reaction)
    const TORSO_DAMP_IDLE = 0.82;   // D: multiplicative damping (1.0 = none)
    const TORSO_DAMP_ACTIVE = 0.92; // Less damping during active control (more momentum)

    // === JOINT REST ANGLES (from split stance geometry) ===
    // These are the relative angles between parent and child at spawn.
    // When locked, joints are driven back toward these angles.
    const LEFT_HIP_REST = HIP_SPLIT;                  //  0.35 (left thigh forward)
    const RIGHT_HIP_REST = -HIP_SPLIT;                // -0.35 (right thigh back)
    const LEFT_KNEE_REST = -KNEE_BEND;                // -0.12 (left knee bends back)
    const RIGHT_KNEE_REST = KNEE_BEND;                //  0.12 (right knee bends forward)
    const LEFT_ANKLE_REST = -(HIP_SPLIT - KNEE_BEND); // -0.23 (keeps left foot flat)
    const RIGHT_ANKLE_REST = (HIP_SPLIT - KNEE_BEND); //  0.23 (keeps right foot flat)

    // === JOINT ANGLE LIMITS ===
    const HIP_MIN_ANGLE = -1.0;
    const HIP_MAX_ANGLE = 1.0;
    const KNEE_MIN_ANGLE = -1.8;
    const KNEE_MAX_ANGLE = 0.5;
    const ANKLE_MIN_ANGLE = -0.8;
    const ANKLE_MAX_ANGLE = 0.8;

    // Constraint stiffness
    const JOINT_STIFFNESS = 1.0;
    const JOINT_DAMPING = 0.5;

    // === ROLLING CARTWHEEL EASTER EGG ===
    const ROLL_TORQUE = 0.6;
    const ROLL_FORWARD_FORCE = 0.001;

    // Collision categories
    const RUNNER_CATEGORY = 0x0002;
    const GROUND_CATEGORY = 0x0001;

    // --- Lock a joint at a target relative angle using velocity-based PD ---
    // Works WITH Matter.js constraints instead of fighting them.
    // D: kills relative angular velocity (makes joint rigid)
    // P: nudges child toward rest angle (corrects drift from gravity)
    function lockJoint(parent, child, restAngle) {
        let relAngle = child.angle - parent.angle;
        while (relAngle > Math.PI) relAngle -= 2 * Math.PI;
        while (relAngle < -Math.PI) relAngle += 2 * Math.PI;
        const relAngVel = child.angularVelocity - parent.angularVelocity;
        const angleError = restAngle - relAngle;

        // D: kill relative velocity
        const dampCorrection = -relAngVel * LOCK_DAMP;
        // P: push toward rest angle
        const posCorrection = angleError * LOCK_POS_GAIN;

        Body.setAngularVelocity(child,
            child.angularVelocity + dampCorrection + posCorrection
        );
    }

    // Soft version of lockJoint for joints that need to flex under motor load.
    // Gently guides toward rest angle but allows significant movement.
    function passiveJoint(parent, child, restAngle) {
        let relAngle = child.angle - parent.angle;
        while (relAngle > Math.PI) relAngle -= 2 * Math.PI;
        while (relAngle < -Math.PI) relAngle += 2 * Math.PI;
        const relAngVel = child.angularVelocity - parent.angularVelocity;
        const angleError = restAngle - relAngle;
        const dampCorrection = -relAngVel * PASSIVE_DAMP;
        const posCorrection = angleError * PASSIVE_POS_GAIN;
        Body.setAngularVelocity(child,
            child.angularVelocity + dampCorrection + posCorrection
        );
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

        // Knee positions
        const leftKneeX = leftHipX + Math.sin(ltAngle) * UPPER_LEG_H;
        const leftKneeY = leftHipY + Math.cos(ltAngle) * UPPER_LEG_H;
        const rightKneeX = rightHipX + Math.sin(rtAngle) * UPPER_LEG_H;
        const rightKneeY = rightHipY + Math.cos(rtAngle) * UPPER_LEG_H;

        // LEFT CALF: thigh angle minus knee bend
        const lcAngle = ltAngle - KNEE_BEND;
        const lcCenterX = leftKneeX + Math.sin(lcAngle) * LOWER_LEG_H / 2;
        const lcCenterY = leftKneeY + Math.cos(lcAngle) * LOWER_LEG_H / 2;
        parts.leftCalf = Bodies.rectangle(lcCenterX, lcCenterY, LOWER_LEG_W, LOWER_LEG_H, {
            ...bodyOptions, density: LIMB_DENSITY, label: 'leftCalf', angle: lcAngle
        });

        // RIGHT CALF: thigh angle PLUS knee bend (toward vertical, mirroring left)
        const rcAngle = rtAngle + KNEE_BEND;
        const rcCenterX = rightKneeX + Math.sin(rcAngle) * LOWER_LEG_H / 2;
        const rcCenterY = rightKneeY + Math.cos(rcAngle) * LOWER_LEG_H / 2;
        parts.rightCalf = Bodies.rectangle(rcCenterX, rcCenterY, LOWER_LEG_W, LOWER_LEG_H, {
            ...bodyOptions, density: LIMB_DENSITY, label: 'rightCalf', angle: rcAngle
        });

        // Ankle positions
        const leftAnkleX = leftKneeX + Math.sin(lcAngle) * LOWER_LEG_H;
        const leftAnkleY = leftKneeY + Math.cos(lcAngle) * LOWER_LEG_H;
        const rightAnkleX = rightKneeX + Math.sin(rcAngle) * LOWER_LEG_H;
        const rightAnkleY = rightKneeY + Math.cos(rcAngle) * LOWER_LEG_H;

        // LEFT FOOT: flat on ground
        parts.leftFoot = Bodies.rectangle(leftAnkleX, leftAnkleY + FOOT_H / 2, FOOT_W, FOOT_H, {
            ...bodyOptions, density: FOOT_DENSITY, friction: FOOT_FRICTION, label: 'leftFoot'
        });

        // RIGHT FOOT: flat on ground
        parts.rightFoot = Bodies.rectangle(rightAnkleX, rightAnkleY + FOOT_H / 2, FOOT_W, FOOT_H, {
            ...bodyOptions, density: FOOT_DENSITY, friction: FOOT_FRICTION, label: 'rightFoot'
        });

        // === CONSTRAINTS (pin joints) ===

        // Neck
        constraints.push(Constraint.create({
            bodyA: parts.head, pointA: { x: 0, y: HEAD_RADIUS * 0.8 },
            bodyB: parts.torso, pointB: { x: 0, y: -TORSO_H / 2 },
            stiffness: 0.9, damping: 0.5, length: 2, label: 'neck',
        }));

        // Head stabilizer
        constraints.push(Constraint.create({
            bodyA: parts.head, pointA: { x: 0, y: 0 },
            bodyB: parts.torso, pointB: { x: 0, y: -TORSO_H / 2 + 5 },
            stiffness: 0.3, damping: 0.3, length: HEAD_RADIUS + 5, label: 'headStabilizer',
        }));

        // Left Hip
        constraints.push(Constraint.create({
            bodyA: parts.torso, pointA: { x: -3, y: TORSO_H / 2 },
            bodyB: parts.leftThigh, pointB: { x: 0, y: -UPPER_LEG_H / 2 },
            stiffness: JOINT_STIFFNESS, damping: JOINT_DAMPING, length: 0, label: 'leftHip',
        }));

        // Right Hip
        constraints.push(Constraint.create({
            bodyA: parts.torso, pointA: { x: 3, y: TORSO_H / 2 },
            bodyB: parts.rightThigh, pointB: { x: 0, y: -UPPER_LEG_H / 2 },
            stiffness: JOINT_STIFFNESS, damping: JOINT_DAMPING, length: 0, label: 'rightHip',
        }));

        // Left Knee
        constraints.push(Constraint.create({
            bodyA: parts.leftThigh, pointA: { x: 0, y: UPPER_LEG_H / 2 },
            bodyB: parts.leftCalf, pointB: { x: 0, y: -LOWER_LEG_H / 2 },
            stiffness: JOINT_STIFFNESS, damping: JOINT_DAMPING, length: 0, label: 'leftKnee',
        }));

        // Right Knee
        constraints.push(Constraint.create({
            bodyA: parts.rightThigh, pointA: { x: 0, y: UPPER_LEG_H / 2 },
            bodyB: parts.rightCalf, pointB: { x: 0, y: -LOWER_LEG_H / 2 },
            stiffness: JOINT_STIFFNESS, damping: JOINT_DAMPING, length: 0, label: 'rightKnee',
        }));

        // Left Ankle
        constraints.push(Constraint.create({
            bodyA: parts.leftCalf, pointA: { x: 0, y: LOWER_LEG_H / 2 },
            bodyB: parts.leftFoot, pointB: { x: -2, y: 0 },
            stiffness: 0.9, damping: 0.4, length: 0, label: 'leftAnkle',
        }));

        // Right Ankle
        constraints.push(Constraint.create({
            bodyA: parts.rightCalf, pointA: { x: 0, y: LOWER_LEG_H / 2 },
            bodyB: parts.rightFoot, pointB: { x: -2, y: 0 },
            stiffness: 0.9, damping: 0.4, length: 0, label: 'rightAnkle',
        }));

        // NOTE: No structural braces. The PD controllers (lockJoint) handle all
        // structural rigidity. This allows motors to actually move the legs
        // through the constraint chain and create ground reaction forces.

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

    // === JOINT MOTOR (TORQUE-BASED, ANGLE-TARGETING) ===
    // Drives joint toward a TARGET ANGLE using torques (not velocity corrections).
    // CRITICAL: Torques persist through the constraint solver, creating real forces
    // that propagate through the joint chain to feet, producing ground reaction.
    // Velocity-based corrections get eaten by the constraint solver.
    // Reaction torque on parent = Newton's 3rd law = torso sway from motor effort.
    const MOTOR_P = 0.3;      // Torque per radian of error (gentle)
    const MOTOR_D = 0.08;     // Torque per rad/s of relative velocity (damping)
    const MOTOR_MAX = 0.5;    // Max torque per frame (prevents explosive flips)

    function driveJoint(parent, child, targetAngle) {
        let relAngle = child.angle - parent.angle;
        while (relAngle > Math.PI) relAngle -= 2 * Math.PI;
        while (relAngle < -Math.PI) relAngle += 2 * Math.PI;
        const relAngVel = child.angularVelocity - parent.angularVelocity;
        const angleError = targetAngle - relAngle;

        let torque = angleError * MOTOR_P - relAngVel * MOTOR_D;
        if (torque > MOTOR_MAX) torque = MOTOR_MAX;
        if (torque < -MOTOR_MAX) torque = -MOTOR_MAX;

        child.torque += torque;
        parent.torque -= torque; // Reaction torque = ground reaction force propagation
    }

    // === ANGLE LIMIT ENFORCEMENT ===
    // Two-part response: (1) kill velocity going further into limit,
    // (2) push back toward limit. Without velocity kill, fast-moving
    // joints blow right through the limit.
    function enforceAngleLimit(parent, child, minAngle, maxAngle) {
        let relAngle = child.angle - parent.angle;
        while (relAngle > Math.PI) relAngle -= 2 * Math.PI;
        while (relAngle < -Math.PI) relAngle += 2 * Math.PI;

        const relVel = child.angularVelocity - parent.angularVelocity;

        if (relAngle < minAngle) {
            const overshoot = minAngle - relAngle;
            // Kill velocity going further into limit
            if (relVel < 0) {
                Body.setAngularVelocity(child, child.angularVelocity - relVel * 0.9);
            }
            // Push back toward limit
            Body.setAngularVelocity(child, child.angularVelocity + overshoot * 0.8);
            Body.setAngularVelocity(parent, parent.angularVelocity - overshoot * 0.2);
        }
        if (relAngle > maxAngle) {
            const overshoot = relAngle - maxAngle;
            // Kill velocity going further into limit
            if (relVel > 0) {
                Body.setAngularVelocity(child, child.angularVelocity - relVel * 0.9);
            }
            // Push back toward limit
            Body.setAngularVelocity(child, child.angularVelocity - overshoot * 0.8);
            Body.setAngularVelocity(parent, parent.angularVelocity + overshoot * 0.2);
        }
    }

    /**
     * Core control function -- called every physics frame.
     *
     * HYBRID APPROACH:
     * - Per-joint PD locks hold relative angles (keeps limbs rigid)
     * - Separate torso PD handles absolute balance (inverted pendulum)
     * - J/R: drive hips with motors, everything else locked
     * - E/K: drive knees with motors, everything else locked
     * - All four: rolling easter egg
     */
    function applyControls(runner, keys) {
        const { torso, leftThigh, rightThigh, leftCalf, rightCalf, leftFoot, rightFoot } = runner.parts;

        const allPressed = keys.j && keys.r && keys.e && keys.k;
        const anyPressed = keys.j || keys.r || keys.e || keys.k;
        const anyHipKey = keys.j || keys.r;
        const anyKneeKey = keys.e || keys.k;
        runner.rolling = allPressed;

        // --- Torso balance PD (absolute world-frame reference) ---
        // Per-joint PD only corrects relative angles. When the whole body
        // tilts as a unit, hip/knee angles don't change. Only this PD
        // references absolute vertical (angle=0).
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
            const hipDelta = phase > 0 ? 0.5 : -0.5;
            driveJoint(torso, leftThigh, hipDelta);
            driveJoint(torso, rightThigh, -hipDelta);
            driveJoint(leftThigh, leftCalf, 0);
            driveJoint(rightThigh, rightCalf, 0);
            driveJoint(leftCalf, leftFoot, 0);
            driveJoint(rightCalf, rightFoot, 0);

            enforceAngleLimit(torso, leftThigh, -2.2, 2.2);
            enforceAngleLimit(torso, rightThigh, -2.2, 2.2);
            enforceAngleLimit(leftThigh, leftCalf, -2.5, 0.5);
            enforceAngleLimit(rightThigh, rightCalf, -2.5, 0.5);
            enforceAngleLimit(leftCalf, leftFoot, -0.6, 0.6);
            enforceAngleLimit(rightCalf, rightFoot, -0.6, 0.6);

            return;
        }

        // --- Determine motor target angles ---
        // Each key shifts the target angle from its rest position.
        // J: left hip swings back, right hip swings forward (scissor)
        // R: left hip swings forward, right hip swings back (opposite scissor)
        // E: left knee straightens, right knee bends
        // K: left knee bends, right knee straightens
        let leftHipAngle = LEFT_HIP_REST;
        let rightHipAngle = RIGHT_HIP_REST;
        let leftKneeAngle = LEFT_KNEE_REST;
        let rightKneeAngle = RIGHT_KNEE_REST;

        if (keys.j && !keys.r) {
            // J = SWAP legs: front leg swings back, back leg swings forward
            // Targets the mirror of the rest position. Both targets are far
            // from the constraint equilibrium (~0), producing real motion.
            leftHipAngle = -HIP_SPLIT;                          // 0.35 -> -0.35 (left swings back)
            rightHipAngle = HIP_SPLIT;                          // -0.35 -> +0.35 (right swings forward)
        } else if (keys.r && !keys.j) {
            // R = EXTEND split: both legs push further from center
            leftHipAngle = LEFT_HIP_REST + HIP_DRIVE_ANGLE;   // +0.35 -> +0.70 (left more forward)
            rightHipAngle = RIGHT_HIP_REST - HIP_DRIVE_ANGLE; // -0.35 -> -0.70 (right more back)
        }

        if (keys.e && !keys.k) {
            leftKneeAngle = LEFT_KNEE_REST + KNEE_DRIVE_ANGLE;  // Left knee extends
            rightKneeAngle = RIGHT_KNEE_REST - KNEE_DRIVE_ANGLE; // Right knee flexes
        } else if (keys.k && !keys.e) {
            leftKneeAngle = LEFT_KNEE_REST - KNEE_DRIVE_ANGLE;  // Left knee flexes
            rightKneeAngle = RIGHT_KNEE_REST + KNEE_DRIVE_ANGLE; // Right knee extends
        }

        // --- HIPS ---
        if (anyHipKey) {
            driveJoint(torso, leftThigh, leftHipAngle);
            driveJoint(torso, rightThigh, rightHipAngle);
        } else {
            lockJoint(torso, leftThigh, LEFT_HIP_REST);
            lockJoint(torso, rightThigh, RIGHT_HIP_REST);
        }

        // --- KNEES ---
        // When hips are active, knees go PASSIVE (soft) so legs can flex.
        // When knees are active, they're driven to target angles.
        // When nothing is active, knees are fully locked.
        if (anyKneeKey) {
            driveJoint(leftThigh, leftCalf, leftKneeAngle);
            driveJoint(rightThigh, rightCalf, rightKneeAngle);
        } else if (anyHipKey) {
            passiveJoint(leftThigh, leftCalf, LEFT_KNEE_REST);
            passiveJoint(rightThigh, rightCalf, RIGHT_KNEE_REST);
        } else {
            lockJoint(leftThigh, leftCalf, LEFT_KNEE_REST);
            lockJoint(rightThigh, rightCalf, RIGHT_KNEE_REST);
        }

        // --- ANKLES ---
        // Passive when any key is active (allows foot to pivot on ground).
        // Fully locked when idle.
        if (anyPressed) {
            passiveJoint(leftCalf, leftFoot, LEFT_ANKLE_REST);
            passiveJoint(rightCalf, rightFoot, RIGHT_ANKLE_REST);
        } else {
            lockJoint(leftCalf, leftFoot, LEFT_ANKLE_REST);
            lockJoint(rightCalf, rightFoot, RIGHT_ANKLE_REST);
        }

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
        const capWhiteColor = '#f0e8dc';
        const capBrimColor = '#2a2a2a';
        const hairColor = '#3b2507';
        const wristbandColor = '#e6cc00';
        const watchColor = '#222222';
        const bottleColor = '#e8e8e8';

        // --- Helper: get world position of a body-local offset ---
        function worldPos(body, lx, ly) {
            const cos = Math.cos(body.angle);
            const sin = Math.sin(body.angle);
            return {
                x: body.position.x + (lx * cos - ly * sin),
                y: body.position.y + (lx * sin + ly * cos),
            };
        }

        // --- Helper: draw a joint connector circle ---
        // Like QWOP's round joint connectors -- bridges the visual gap
        // between body parts so the character looks connected.
        function drawJointCircle(body, offsetX, offsetY, radius, color) {
            const p = worldPos(body, offsetX, offsetY);
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.arc(p.x, p.y, radius, 0, Math.PI * 2);
            ctx.fill();
        }

        // Joint connector radii (sized to bridge gaps between parts)
        const hipR = 6;    // covers gap between torso and thigh
        const kneeR = 5.5; // covers gap between thigh and calf
        const ankleR = 5;  // covers gap between calf and foot

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
        drawJointCircle(parts.torso, -3, TORSO_H / 2, hipR, skinShadow);
        drawBodyPart(parts.leftThigh, UPPER_LEG_W, UPPER_LEG_H, skinShadow);
        drawJointCircle(parts.leftThigh, 0, UPPER_LEG_H / 2, kneeR, skinShadow);
        drawBodyPart(parts.leftCalf, LOWER_LEG_W, LOWER_LEG_H, skinShadow);
        drawJointCircle(parts.leftCalf, 0, LOWER_LEG_H / 2, ankleR, skinShadow);
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
        drawJointCircle(parts.torso, 3, TORSO_H / 2, hipR, skinColor);
        drawBodyPart(parts.rightThigh, UPPER_LEG_W, UPPER_LEG_H, skinColor);
        drawJointCircle(parts.rightThigh, 0, UPPER_LEG_H / 2, kneeR, skinColor);
        drawBodyPart(parts.rightCalf, LOWER_LEG_W, LOWER_LEG_H, skinColor);
        drawJointCircle(parts.rightCalf, 0, LOWER_LEG_H / 2, ankleR, skinColor);
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

        // --- Running cap (redesigned) ---
        // Runner faces RIGHT. Red crown covers back/top, white front panel faces right.
        const R = HEAD_RADIUS + 1;
        const capY = -2;

        // Red crown (back and top)
        ctx.fillStyle = capRedColor;
        ctx.beginPath();
        ctx.arc(0, capY, R, -Math.PI * 0.85, -Math.PI * 0.1);
        ctx.lineTo(R * 0.25, capY - R - 2);
        ctx.closePath();
        ctx.fill();

        // White front panel (facing right)
        ctx.fillStyle = capWhiteColor;
        ctx.beginPath();
        ctx.arc(0, capY, R, -Math.PI * 0.1, Math.PI * 0.08);
        ctx.lineTo(R * 0.25, capY - R - 2);
        ctx.closePath();
        ctx.fill();

        // CLIF logo on front panel (red text on white)
        ctx.fillStyle = '#cc2222';
        ctx.font = 'bold 4px Arial';
        ctx.textAlign = 'center';
        ctx.fillText('CLIF', 3, capY - HEAD_RADIUS + 5);

        // Brim (extends forward/right)
        ctx.fillStyle = capBrimColor;
        ctx.beginPath();
        ctx.moveTo(HEAD_RADIUS * 0.6, -HEAD_RADIUS * 0.35);
        ctx.quadraticCurveTo(HEAD_RADIUS + 9, -HEAD_RADIUS * 0.45, HEAD_RADIUS + 12, -HEAD_RADIUS * 0.1);
        ctx.quadraticCurveTo(HEAD_RADIUS + 9, HEAD_RADIUS * 0.1, HEAD_RADIUS * 0.5, -HEAD_RADIUS * 0.05);
        ctx.closePath();
        ctx.fill();

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

        ctx.restore();
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
