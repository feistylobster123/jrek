/**
 * JREK UI - HUD, menus, fall messages, finish screen
 */

const UI = (function() {

    // Fall messages - shown when the runner faceplants
    const FALL_MESSAGES = [
        'MINNESOTA!!',
        'Have you tried being vegan?',
        'Even flatlanders fall sometimes',
        'Scott would never',
        "That's not a barrel roll",
        'Eat & Run... emphasis on the run part',
        'Tim Twietmeyer is watching',
        'DNF: Did Not Feet',
        'The trail always wins',
        '15:36 course record pace this is not',
        'Somewhere, Dean Karnazes is laughing',
        'Born to Run. Forgot how.',
        'Ultra DNF',
        'Western States? More like Western Flails',
        'This is harder than Hardrock',
        'Maybe try QWOP first',
        'Your Strava followers are concerned',
        'Aid station! Someone get ice!',
        'That bandana deserves better',
    ];

    // State
    let titleScreenVisible = true;
    let fallMessageEl = null;
    let fallMessageTimeout = null;
    let restartPromptEl = null;
    let finishScreenEl = null;
    let confettiCtx = null;
    let confettiParticles = [];

    function init() {
        fallMessageEl = document.getElementById('fallMessage');
        finishScreenEl = document.getElementById('finishScreen');
    }

    function hideTitleScreen() {
        const el = document.getElementById('titleScreen');
        if (el) {
            el.style.transition = 'opacity 0.5s ease';
            el.style.opacity = '0';
            setTimeout(() => {
                el.style.display = 'none';
            }, 500);
        }
        titleScreenVisible = false;
    }

    function isTitleVisible() {
        return titleScreenVisible;
    }

    // Show a random fall message
    function showFallMessage() {
        if (!fallMessageEl) return;

        const msg = FALL_MESSAGES[Math.floor(Math.random() * FALL_MESSAGES.length)];
        fallMessageEl.textContent = msg;
        fallMessageEl.classList.remove('hidden', 'fade-out');

        // Clear any existing timeout
        if (fallMessageTimeout) clearTimeout(fallMessageTimeout);

        // Fade out after 2.5 seconds
        fallMessageTimeout = setTimeout(() => {
            fallMessageEl.classList.add('fade-out');
        }, 2500);
    }

    function hideFallMessage() {
        if (fallMessageEl) {
            fallMessageEl.classList.add('hidden');
        }
    }

    // Show restart prompt
    function showRestartPrompt() {
        if (restartPromptEl) return;
        restartPromptEl = document.createElement('div');
        restartPromptEl.className = 'restart-prompt';
        const isMobile = ('ontouchstart' in window) || (navigator.maxTouchPoints > 0);
        restartPromptEl.textContent = isMobile ? 'Tap to try again' : 'Press SPACE to try again';
        document.body.appendChild(restartPromptEl);
    }

    function hideRestartPrompt() {
        if (restartPromptEl) {
            restartPromptEl.remove();
            restartPromptEl = null;
        }
    }

    // Show finish screen with confetti
    function showFinishScreen() {
        if (!finishScreenEl) return;
        finishScreenEl.classList.remove('hidden');

        const confettiCanvas = document.getElementById('confettiCanvas');
        confettiCanvas.width = window.innerWidth;
        confettiCanvas.height = window.innerHeight;
        confettiCtx = confettiCanvas.getContext('2d');

        // Create confetti particles
        const colors = ['#ffd700', '#ff6347', '#4169e1', '#32cd32', '#ff69b4', '#ffa500'];
        for (let i = 0; i < 200; i++) {
            confettiParticles.push({
                x: Math.random() * confettiCanvas.width,
                y: Math.random() * confettiCanvas.height - confettiCanvas.height,
                vx: (Math.random() - 0.5) * 4,
                vy: Math.random() * 3 + 2,
                size: Math.random() * 8 + 4,
                color: colors[Math.floor(Math.random() * colors.length)],
                rotation: Math.random() * Math.PI * 2,
                rotSpeed: (Math.random() - 0.5) * 0.2,
            });
        }

        animateConfetti();
    }

    function animateConfetti() {
        if (!confettiCtx) return;
        const canvas = confettiCtx.canvas;
        confettiCtx.clearRect(0, 0, canvas.width, canvas.height);

        confettiParticles.forEach(p => {
            p.x += p.vx;
            p.y += p.vy;
            p.rotation += p.rotSpeed;
            p.vy += 0.05;

            if (p.y > canvas.height + 20) {
                p.y = -20;
                p.x = Math.random() * canvas.width;
                p.vy = Math.random() * 3 + 2;
            }

            confettiCtx.save();
            confettiCtx.translate(p.x, p.y);
            confettiCtx.rotate(p.rotation);
            confettiCtx.fillStyle = p.color;
            confettiCtx.fillRect(-p.size / 2, -p.size / 4, p.size, p.size / 2);
            confettiCtx.restore();
        });

        requestAnimationFrame(animateConfetti);
    }

    // Render HUD on the game canvas
    function renderHUD(ctx, canvasWidth, canvasHeight, state) {
        const { distance, miles, pace, elapsed, fallen, currentMarker, nextMarker, keys } = state;

        ctx.save();

        // HUD background strip at top
        ctx.fillStyle = 'rgba(0, 0, 0, 0.5)';
        ctx.fillRect(0, 0, canvasWidth, 52);

        ctx.font = 'bold 14px "Courier New", monospace';
        ctx.textAlign = 'left';

        // Distance
        ctx.fillStyle = '#e8d5b7';
        ctx.fillText(`Distance: ${miles.toFixed(3)} mi`, 15, 20);

        // Pace (if moving)
        if (pace > 0) {
            const paceMin = Math.floor(pace);
            const paceSec = Math.floor((pace - paceMin) * 60);
            ctx.fillText(`Pace: ${paceMin}:${paceSec.toString().padStart(2, '0')} /mi`, 15, 38);
        } else {
            ctx.fillText('Pace: --:-- /mi', 15, 38);
        }

        // Timer
        ctx.textAlign = 'center';
        const totalSec = Math.floor(elapsed / 1000);
        const hrs = Math.floor(totalSec / 3600);
        const mins = Math.floor((totalSec % 3600) / 60);
        const secs = totalSec % 60;
        const timeStr = `${hrs}:${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
        ctx.fillStyle = '#ffd700';
        ctx.font = 'bold 18px "Courier New", monospace';
        ctx.fillText(timeStr, canvasWidth / 2, 22);

        // Location
        ctx.font = '11px "Courier New", monospace';
        ctx.fillStyle = '#a89070';
        if (currentMarker) {
            ctx.fillText(currentMarker.name, canvasWidth / 2, 40);
        }

        // Next marker
        ctx.textAlign = 'right';
        ctx.fillStyle = '#8a7a6a';
        ctx.font = '12px "Courier New", monospace';
        if (nextMarker) {
            const dist = nextMarker.mile - miles;
            ctx.fillText(`Next: ${nextMarker.name} (${dist.toFixed(1)} mi)`, canvasWidth - 15, 20);
        }

        // Distance in feet for early game
        if (miles < 0.1) {
            const feet = miles * 5280;
            ctx.fillStyle = '#c0a888';
            ctx.textAlign = 'right';
            ctx.fillText(`(${feet.toFixed(0)} ft)`, canvasWidth - 15, 38);
        }

        // Version tag (bottom-left, subtle)
        ctx.textAlign = 'left';
        ctx.fillStyle = 'rgba(255,255,255,0.2)';
        ctx.font = '9px "Courier New", monospace';
        ctx.fillText('v0.9', 6, canvasHeight - 6);

        // Control indicators at bottom
        renderControlIndicators(ctx, canvasWidth, canvasHeight, keys);

        ctx.restore();
    }

    function renderControlIndicators(ctx, canvasWidth, canvasHeight, keys) {
        const y = canvasHeight - 30;
        const centerX = canvasWidth / 2;
        const spacing = 50;
        const keyLabels = [
            { key: 'j', label: 'J', desc: 'L.Thigh' },
            { key: 'e', label: 'E', desc: 'L.Calf' },
            { key: 'k', label: 'K', desc: 'R.Calf' },
            { key: 'r', label: 'R', desc: 'R.Thigh' },
        ];

        ctx.textAlign = 'center';

        keyLabels.forEach((k, i) => {
            const kx = centerX - (keyLabels.length / 2 - 0.5) * spacing + i * spacing;
            const active = keys[k.key];

            // Key box
            ctx.fillStyle = active ? 'rgba(232,213,183,0.9)' : 'rgba(232,213,183,0.2)';
            ctx.beginPath();
            ctx.roundRect(kx - 15, y - 12, 30, 24, 4);
            ctx.fill();

            ctx.strokeStyle = active ? '#ffd700' : 'rgba(232,213,183,0.4)';
            ctx.lineWidth = active ? 2 : 1;
            ctx.beginPath();
            ctx.roundRect(kx - 15, y - 12, 30, 24, 4);
            ctx.stroke();

            // Key letter
            ctx.fillStyle = active ? '#2a1f14' : 'rgba(232,213,183,0.6)';
            ctx.font = `bold 14px "Courier New", monospace`;
            ctx.fillText(k.label, kx, y + 5);

            // Description below
            ctx.fillStyle = 'rgba(232,213,183,0.4)';
            ctx.font = '8px "Courier New", monospace';
            ctx.fillText(k.desc, kx, y + 22);
        });
    }

    // Render a "starting" message
    function renderStartMessage(ctx, canvasWidth, canvasHeight, elapsed) {
        if (elapsed > 3000) return;

        const alpha = Math.max(0, 1 - elapsed / 3000);
        ctx.save();
        ctx.globalAlpha = alpha;
        ctx.fillStyle = '#e8d5b7';
        ctx.font = 'bold 16px "Courier New", monospace';
        ctx.textAlign = 'center';
        ctx.fillText('Squaw Valley, elevation 6,200 ft', canvasWidth / 2, canvasHeight / 2 - 60);
        ctx.font = '13px "Courier New", monospace';
        ctx.fillText('100 miles to Auburn.', canvasWidth / 2, canvasHeight / 2 - 38);
        ctx.restore();
    }

    // Render personal best display
    function renderPersonalBest(ctx, canvasWidth, bestMiles) {
        if (bestMiles <= 0) return;
        ctx.save();
        ctx.textAlign = 'right';
        ctx.fillStyle = 'rgba(168, 144, 112, 0.6)';
        ctx.font = '10px "Courier New", monospace';
        const bestFeet = bestMiles * 5280;
        if (bestMiles < 0.1) {
            ctx.fillText(`Best: ${bestFeet.toFixed(0)} ft`, canvasWidth - 15, 48);
        } else {
            ctx.fillText(`Best: ${bestMiles.toFixed(3)} mi`, canvasWidth - 15, 48);
        }
        ctx.restore();
    }

    return {
        init,
        hideTitleScreen,
        isTitleVisible,
        showFallMessage,
        hideFallMessage,
        showRestartPrompt,
        hideRestartPrompt,
        showFinishScreen,
        renderHUD,
        renderStartMessage,
        renderPersonalBest,
    };
})();
