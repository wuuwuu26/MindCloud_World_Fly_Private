/*
 * Copyright 2026 Manifold Tech Ltd.
 * Author: MENG Guotao <mengguotao@manifoldtech.cn>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * HUD overlay — updates HTML elements with drone telemetry, FPS, controller status.
 */

import { formatLap } from './gates.js';

export class HUD {
    constructor() {
        this.altitudeEl = document.getElementById('hud-altitude');
        this.vspeedEl = document.getElementById('hud-vspeed');
        this.gspeedEl = document.getElementById('hud-gspeed');
        this.fpsEl = document.getElementById('hud-fps');
        this.controllerEl = document.getElementById('hud-controller');
        this.collisionWarnEl = document.getElementById('hud-collision-warn');
        this.armedEl = document.getElementById('armed-indicator');
        this.collisionFlashEl = document.getElementById('collision-flash');
        this.raceProgressEl = document.getElementById('hud-race-progress');
        this.lapTimerEl = document.getElementById('hud-lap-timer');
        this.hudContainer = document.getElementById('hud');

        // FPS tracking
        this._frameTimes = [];
        this._lastTime = performance.now();
        this._collisionFlashTimer = 0;

        // Race HUD flash: briefly tint green + enlarge on each gate-pass.
        // The timer counts down in milliseconds inside update().
        this._raceFlashTimer = 0;
        this._raceLastPassed = 0;
    }

    show() {
        this.hudContainer.classList.remove('hidden');
    }

    hide() {
        this.hudContainer.classList.add('hidden');
    }

    /**
     * Update HUD each frame.
     * @param {Drone} drone
     * @param {Controller} controller
     * @param {GateCourse} [gateCourse] - optional; when present and
     *   enabled with at least one placed gate, renders a
     *   "Gate X / N" progress readout in the bottom-center HUD zone.
     */
    update(drone, controller, gateCourse) {
        const now = performance.now();
        const dt = now - this._lastTime;
        this._lastTime = now;

        // FPS calculation (rolling average)
        this._frameTimes.push(dt);
        if (this._frameTimes.length > 60) this._frameTimes.shift();
        const avgDt = this._frameTimes.reduce((a, b) => a + b, 0) / this._frameTimes.length;
        const fps = Math.round(1000 / avgDt);

        // Update values
        if (this.altitudeEl) this.altitudeEl.textContent = drone.y.toFixed(1);
        if (this.vspeedEl) {
            this.vspeedEl.textContent = drone.verticalSpeed.toFixed(1);
            this.vspeedEl.style.color = drone.verticalSpeed < -2 ? '#f80' : '#0f0';
        }
        if (this.gspeedEl) this.gspeedEl.textContent = drone.speed.toFixed(1);
        if (this.fpsEl) this.fpsEl.textContent = fps;

        // Controller status
        if (this.controllerEl) {
            if (controller.connected) {
                this.controllerEl.textContent = 'RC Connected';
                this.controllerEl.style.color = '#4272F5';
            } else {
                this.controllerEl.textContent = 'Keyboard';
                this.controllerEl.style.color = '#0f0';
            }
        }

        // Armed indicator
        if (this.armedEl) {
            if (controller.armed) {
                this.armedEl.textContent = 'ARMED';
                this.armedEl.className = 'armed';
            } else {
                this.armedEl.textContent = 'DISARMED';
                this.armedEl.className = 'disarmed';
            }
        }

        // Collision warning
        if (drone.isColliding) {
            this._collisionFlashTimer = 150; // ms
            if (this.collisionWarnEl) this.collisionWarnEl.style.display = 'block';
        } else {
            if (this.collisionWarnEl) this.collisionWarnEl.style.display = 'none';
        }

        // Collision flash effect
        if (this._collisionFlashTimer > 0) {
            this._collisionFlashTimer -= dt;
            if (this.collisionFlashEl) {
                this.collisionFlashEl.classList.add('active');
                this.collisionFlashEl.style.opacity = Math.min(1, drone.collisionIntensity * 0.5);
            }
        } else {
            if (this.collisionFlashEl) {
                this.collisionFlashEl.classList.remove('active');
            }
        }

        // Race-course readouts. Two elements in the bottom-centre zone:
        //   hud-lap-timer     — "Lap 3 · 00:42.314 · best 00:39.120"
        //   hud-race-progress — "Gate 2 / 7"
        // Both hidden unless gateMode is on (isVisible()) AND a path of
        // >= 3 gates exists. Out-of-order gate crossings are silently
        // ignored by the course, so there is no "missed lap" state to
        // surface here.
        const hasCourse = !!(gateCourse &&
                             typeof gateCourse.isVisible === 'function' &&
                             gateCourse.isVisible() &&
                             gateCourse.gates && gateCourse.gates.length >= 3);

        if (this.raceProgressEl) {
            if (hasCourse) {
                const passed = gateCourse.passedCount();
                const total  = gateCourse.gates.length;

                if (passed > this._raceLastPassed) this._raceFlashTimer = 400;
                this._raceLastPassed = passed;

                const next = ((gateCourse.nextGateIdx | 0) % total) + 1;
                this.raceProgressEl.textContent = `Gate ${next} / ${total}`;
                this.raceProgressEl.style.display = 'block';

                if (this._raceFlashTimer > 0) {
                    this._raceFlashTimer -= dt;
                    const t = Math.max(0, this._raceFlashTimer) / 400;
                    this.raceProgressEl.style.color     = `rgb(${Math.round(77 + 178 * t)}, ${Math.round(221 + 34 * t)}, ${Math.round(255 - 80 * t)})`;
                    this.raceProgressEl.style.transform = `scale(${1 + 0.15 * t})`;
                } else {
                    this.raceProgressEl.style.color     = '#4df';
                    this.raceProgressEl.style.transform = 'scale(1)';
                }
            } else {
                this.raceProgressEl.style.display = 'none';
                this._raceLastPassed = 0;
                this._raceFlashTimer = 0;
            }
        }

        if (this.lapTimerEl) {
            if (hasCourse) {
                const lapStartedYet = gateCourse.lapStart != null;
                const lapMs   = lapStartedYet ? gateCourse.currentLapMs : 0;
                const bestMs  = gateCourse.bestLapMs;
                const lapNum  = gateCourse.lapCount + 1;

                let text;
                if (!lapStartedYet) {
                    text = `Cross gate 1 to start the timer${bestMs != null ? ` · best ${formatLap(bestMs)}` : ''}`;
                } else {
                    const bestBit = (bestMs != null) ? ` · best ${formatLap(bestMs)}` : '';
                    text = `Lap ${lapNum} · ${formatLap(lapMs)}${bestBit}`;
                }
                this.lapTimerEl.textContent = text;
                this.lapTimerEl.style.display = 'block';
                this.lapTimerEl.style.color = lapStartedYet ? '#4df' : '#aaa';
            } else {
                this.lapTimerEl.style.display = 'none';
            }
        }
    }
}
