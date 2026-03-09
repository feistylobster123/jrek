/**
 * JREK Runner - Ragdoll physics body for Scott Jurek
 * Built with Matter.js rigid bodies and constraints
 *
 * Physics: Authentic QWOP-style joint motor simulation.
 * Each key pair (J/R for hips, E/K for knees) drives BOTH legs
 * in opposite directions, exactly like QWOP's Q/W and O/P.
 *
 * Key mechanics:
 * - Joint motors target an angular velocity, limited by max impulse
 * - When no key is pressed, motors brake (resist rotation = stiff joints)
 * - No active torso stabilization — balance from coordinated leg motion
 * - Very heavy feet provide passive pendulum stability
 * - Joint angle limits prevent unrealistic hyperextension
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

    // === MASS DISTRIBUTION (QWOP-inspired) ===
    // Heavy torso + very heavy feet = low center of mass = pendulum stability
    const TORSO_DENSITY = 0.006;    // Heaviest segment (like real human)
    const LIMB_DENSITY = 0.001;     // Light limbs
    const FOOT_DENSITY = 0.02;      // 20x limb density (QWOP's key insight)
    const HEAD_DENSITY = 0.0003;    // Near-massless (just for visual/collision)

    // === SURFACE FRICTION ===
    const BODY_FRICTION = 0.6;
    const BODY_RESTITUTION = 0.02;
    const FOOT_FRICTION = 5.0;      // Very high — feet grip the trail (QWOP uses 5.0)

    // === JOINT MOTOR PARAMETERS ===
    // Motors drive relative angular velocity between connected bodies.
    // When active (key pressed): motor drives toward target speed.
    // When inactive (key released): motor brakes (drives toward speed 0).
    const HIP_MOTOR_SPEED = 0.15;   // Target relative angular velocity (rad/frame)
    const KNEE_MOTOR_SPEED = 0.15;  // Target relative angular velocity (rad/frame)
    const MOTOR_STRENGTH = 0.3;     // How aggressively motor reaches target (0-1)
    const BRAKE_STRENGTH = 0.15;    // How aggressively motor brakes (0-1)

    // === JOINT ANGLE LIMITS ===
    // Relative angle of child body to parent body.
    // Positive = clockwise rotation in screen coords (y-down).
    const HIP_MIN_ANGLE = -1.2;     // ~-69° (leg behind torso)
    const HIP_MAX_ANGLE = 1.2;      // ~+69° (leg in front of torso)
    const KNEE_MIN_ANGLE = -2.0;    // ~-115° (fully bent, heel toward butt)
    const KNEE_MAX_ANGLE = 0.1;     // Slightly past straight (tiny tolerance)
    const ANKLE_MIN_ANGLE = -0.25;  // Nearly rigid
    const ANKLE_MAX_ANGLE = 0.25;   // Nearly rigid

    // === DAMPING ===
    const ANGULAR_DAMPING = 0.97;   // Per-frame angular velocity decay (prevents wild spinning)

    // Constraint stiffness
    const JOINT_STIFFNESS = 1.0;
    const JOINT_DAMPING = 0.5;

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
            frictionAir: 0.01,
        };

        // HEAD
        parts.head = Bodies.circle(x, y - TORSO_H / 2 - HEAD_RADIUS - 2, HEAD_RADIUS, {
            ...bodyOptions,
            density: HEAD_DENSITY,
            label: 'head',
        });

        // TORSO — heavy, pure ragdoll. Low air friction so it falls naturally.
        // No hidden stabilization -- this is the inverted pendulum.
        parts.torso = Bodies.rectangle(x, y, TORSO_W, TORSO_H, {
            ...bodyOptions,
            density: TORSO_DENSITY,
            label: 'torso',
            frictionAir: 0.015,
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

        // LEFT FOOT — very heavy for stability
        parts.leftFoot = Bodies.rectangle(
            x - 3, y + TORSO_H / 2 + UPPER_LEG_H + LOWER_LEG_H + FOOT_H / 2,
            FOOT_W, FOOT_H,
            { ...bodyOptions, density: FOOT_DENSITY, friction: FOOT_FRICTION, label: 'leftFoot' }
        );

        // RIGHT FOOT — very heavy for stability
        parts.rightFoot = Bodies.rectangle(
            x + 3, y + TORSO_H / 2 + UPPER_LEG_H + LOWER_LEG_H + FOOT_H / 2,
            FOOT_W, FOOT_H,
            { ...bodyOptions, density: FOOT_DENSITY, friction: FOOT_FRICTION, label: 'rightFoot' }
        );

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

        // === STRUCTURAL SAFETY NET ===
        // Thigh cross-brace prevents doing the splits (unrealistic in Matter.js)
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

    // === JOINT MOTOR ===
    // Simulates a Box2D-style revolute joint motor in Matter.js.
    // Drives relative angular velocity between parent and child toward targetSpeed.
    // When targetSpeed is 0, acts as a brake (resists rotation = stiff joint).
    function applyJointMotor(parent, child, targetSpeed) {
        const relAngVel = child.angularVelocity - parent.angularVelocity;
        const isActive = targetSpeed !== 0;
        const strength = isActive ? MOTOR_STRENGTH : BRAKE_STRENGTH;

        const error = targetSpeed - relAngVel;
        const impulse = error * strength;

        // Distribute impulse by inverse inertia (lighter body moves more)
        // This is Newton's 3rd law — equal and opposite torques, but
        // angular acceleration = torque / inertia, so lighter body accelerates more.
        const totalI = parent.inertia + child.inertia;
        const childShare = parent.inertia / totalI;
        const parentShare = child.inertia / totalI;

        Body.setAngularVelocity(child, child.angularVelocity + impulse * childShare);
        Body.setAngularVelocity(parent, parent.angularVelocity - impulse * parentShare);
    }

    // === ANGLE LIMIT ENFORCEMENT ===
    // Matter.js doesn't have native angle limits on constraints.
    // This manually prevents joints from exceeding their range of motion.
    function enforceAngleLimit(parent, child, minAngle, maxAngle) {
        let relAngle = child.angle - parent.angle;
        // Normalize to [-PI, PI]
        while (relAngle > Math.PI) relAngle -= 2 * Math.PI;
        while (relAngle < -Math.PI) relAngle += 2 * Math.PI;

        if (relAngle < minAngle) {
            const overshoot = minAngle - relAngle;
            // Strong corrective push
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
     * Core control function — called every physics frame.
     *
     * QWOP-style paired controls:
     * - J: left hip backward + right hip forward (like QWOP's Q)
     * - R: left hip forward + right hip backward (like QWOP's W)
     * - E: left knee one direction + right knee other (like QWOP's O)
     * - K: opposite of E (like QWOP's P)
     *
     * When no key is pressed for a joint pair, the motor brakes
     * (drives speed toward 0), keeping joints stiff.
     */
    // === ROLLING CARTWHEEL EASTER EGG PARAMETERS ===
    const ROLL_SPIN_TARGET = 0.18;  // Target torso angular velocity (rad/frame, ~1.7 rps)
    const ROLL_SPIN_STRENGTH = 0.12; // How aggressively we reach target spin
    const ROLL_FORWARD_FORCE = 0.002; // Horizontal push to keep moving forward
    const ROLL_DOWN_FORCE = 0.0005;   // Gentle downward pull for ground contact

    function applyControls(runner, keys) {
        const { torso, leftThigh, rightThigh, leftCalf, rightCalf, leftFoot, rightFoot } = runner.parts;

        const allPressed = keys.j && keys.r && keys.e && keys.k;
        runner.rolling = allPressed;

        // --- Angular damping on all bodies (prevents perpetual spinning) ---
        // Matter.js frictionAir only damps linear velocity, not angular.
        // Reduce torso damping during roll so it sustains the spin.
        for (const key of Object.keys(runner.parts)) {
            const part = runner.parts[key];
            const damping = (allPressed && part === torso) ? 0.995 : ANGULAR_DAMPING;
            Body.setAngularVelocity(part, part.angularVelocity * damping);
        }

        // === Easter egg: all four keys = sloppy rolling cartwheel ===
        if (allPressed) {
            // Drive torso toward a sustained forward spin (clockwise in screen coords)
            const spinError = ROLL_SPIN_TARGET - torso.angularVelocity;
            Body.setAngularVelocity(torso, torso.angularVelocity + spinError * ROLL_SPIN_STRENGTH);

            // Gentle forward + downward push. The heavy feet hitting the ground
            // create most of the forward propulsion; this just keeps things moving.
            Body.applyForce(torso, torso.position, {
                x: ROLL_FORWARD_FORCE,
                y: ROLL_DOWN_FORCE,
            });

            // Windmill leg movement: alternate which leg extends forward vs back
            // based on the torso's rotation phase. Creates the spoke/windmill look.
            const phase = Math.sin(torso.angle * 2);
            applyJointMotor(torso, leftThigh, phase > 0 ? HIP_MOTOR_SPEED : -HIP_MOTOR_SPEED);
            applyJointMotor(torso, rightThigh, phase > 0 ? -HIP_MOTOR_SPEED : HIP_MOTOR_SPEED);

            // Knees mostly straight so legs extend outward like windmill blades
            applyJointMotor(leftThigh, leftCalf, KNEE_MOTOR_SPEED * 0.4);
            applyJointMotor(rightThigh, rightCalf, KNEE_MOTOR_SPEED * 0.4);

            // Let ankles go loose
            applyJointMotor(leftCalf, leftFoot, 0);
            applyJointMotor(rightCalf, rightFoot, 0);

            // Wider angle limits during roll so limbs can flail freely
            enforceAngleLimit(torso, leftThigh, -2.2, 2.2);
            enforceAngleLimit(torso, rightThigh, -2.2, 2.2);
            enforceAngleLimit(leftThigh, leftCalf, -2.5, 0.5);
            enforceAngleLimit(rightThigh, rightCalf, -2.5, 0.5);
            enforceAngleLimit(leftCalf, leftFoot, -0.6, 0.6);
            enforceAngleLimit(rightCalf, rightFoot, -0.6, 0.6);

            return; // Skip normal controls
        }

        // --- Determine target motor speeds ---
        // Each key drives BOTH legs of the pair in opposite directions.
        // This creates the natural reciprocal motion of running.
        let leftHipTarget = 0;
        let rightHipTarget = 0;
        let leftKneeTarget = 0;
        let rightKneeTarget = 0;

        // J: swing right thigh forward, left thigh backward
        // R: swing left thigh forward, right thigh backward
        if (keys.j && !keys.r) {
            leftHipTarget = -HIP_MOTOR_SPEED;
            rightHipTarget = HIP_MOTOR_SPEED;
        } else if (keys.r && !keys.j) {
            leftHipTarget = HIP_MOTOR_SPEED;
            rightHipTarget = -HIP_MOTOR_SPEED;
        }
        // Both pressed or neither = brake (target speed 0)

        // E/K: swing calves in opposite directions
        if (keys.e && !keys.k) {
            leftKneeTarget = KNEE_MOTOR_SPEED;
            rightKneeTarget = -KNEE_MOTOR_SPEED;
        } else if (keys.k && !keys.e) {
            leftKneeTarget = -KNEE_MOTOR_SPEED;
            rightKneeTarget = KNEE_MOTOR_SPEED;
        }

        // --- Apply joint motors ---
        applyJointMotor(torso, leftThigh, leftHipTarget);
        applyJointMotor(torso, rightThigh, rightHipTarget);
        applyJointMotor(leftThigh, leftCalf, leftKneeTarget);
        applyJointMotor(rightThigh, rightCalf, rightKneeTarget);

        // Ankles: always brake (keep feet relatively flat)
        applyJointMotor(leftCalf, leftFoot, 0);
        applyJointMotor(rightCalf, rightFoot, 0);

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
        // Can't fall while doing the barrel roll
        if (runner.rolling) return false;

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

        // Black shorts with gear belt - longer, extending past torso bottom
        const shortsTop = TORSO_H / 4;
        const shortsLength = TORSO_H / 2 - shortsTop + 8; // +8px extends past torso
        ctx.fillStyle = shortsColor;
        ctx.beginPath();
        ctx.roundRect(-TORSO_W / 2 - 2, shortsTop, TORSO_W + 4, shortsLength, [0, 0, 3, 3]);
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

        // CLIF trucker cap - centered on top of head
        // Cap dome (red front panel) - covers top of head from back to front
        ctx.fillStyle = capRedColor;
        ctx.beginPath();
        // Arc sitting on top of the head, centered
        ctx.arc(0, -2, HEAD_RADIUS + 1, Math.PI * 1.15, Math.PI * 1.85);
        // Crown bump
        ctx.quadraticCurveTo(0, -HEAD_RADIUS - 6, 0, -HEAD_RADIUS - 4);
        ctx.closePath();
        ctx.fill();

        // Cap mesh (white/grey back panels)
        ctx.fillStyle = capWhiteColor;
        ctx.beginPath();
        ctx.arc(0, -2, HEAD_RADIUS + 1, Math.PI * 0.15, Math.PI * 0.35);
        ctx.lineTo(0, -HEAD_RADIUS - 4);
        ctx.arc(0, -2, HEAD_RADIUS + 1, Math.PI * 1.85, Math.PI * 2.0);
        ctx.closePath();
        ctx.fill();

        // Cap brim - pointing right (forward), centered on forehead
        ctx.fillStyle = capBrimColor;
        ctx.beginPath();
        ctx.moveTo(HEAD_RADIUS * 0.5, -HEAD_RADIUS * 0.5);
        ctx.quadraticCurveTo(HEAD_RADIUS + 8, -HEAD_RADIUS * 0.6, HEAD_RADIUS + 11, -HEAD_RADIUS * 0.25);
        ctx.quadraticCurveTo(HEAD_RADIUS + 8, HEAD_RADIUS * 0.0, HEAD_RADIUS * 0.4, -HEAD_RADIUS * 0.15);
        ctx.closePath();
        ctx.fill();

        // CLIF text on cap front (tiny)
        ctx.fillStyle = '#ffffff';
        ctx.font = 'bold 5px Arial';
        ctx.textAlign = 'center';
        ctx.fillText('CLIF', 0, -HEAD_RADIUS + 2);

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
        } else if (runner.rolling) {
            // Manic grin during barrel roll
            ctx.moveTo(-5, 4);
            ctx.quadraticCurveTo(0, 9, 5, 4);
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
