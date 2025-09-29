/**
 * UI Controller
 *
 * This module manages the user interface interactions and coordinates
 * between the UI elements and the XUM1541/CBM protocol layers.
 */

class UIController {
    constructor() {
        this.xum1541 = null;
        this.cbmProtocol = null;
        this.d64Handler = null;
        this.currentOperation = null;

        this.elements = {};
        this.initializeElements();
        this.bindEvents();
        this.updateConnectionStatus(false);
    }

    /**
     * Initialize DOM element references
     */
    initializeElements() {
        // Connection elements
        this.elements.connectBtn = document.getElementById('connect-btn');
        this.elements.disconnectBtn = document.getElementById('disconnect-btn');
        this.elements.connectionStatus = document.getElementById('connection-status');
        this.elements.connectionText = document.getElementById('connection-text');
        this.elements.deviceInfo = document.getElementById('device-info');
        this.elements.deviceName = document.getElementById('device-name');
        this.elements.firmwareVersion = document.getElementById('firmware-version');
        this.elements.deviceCapabilities = document.getElementById('device-capabilities');

        // Drive operation elements
        this.elements.driveSection = document.getElementById('drive-section');
        this.elements.driveSelect = document.getElementById('drive-select');
        this.elements.readDirBtn = document.getElementById('read-dir-btn');
        this.elements.directoryDisplay = document.getElementById('directory-display');
        this.elements.resetDriveBtn = document.getElementById('reset-drive-btn');

        // D64 operation elements
        this.elements.d64FileInput = document.getElementById('d64-file-input');
        this.elements.selectedFileName = document.getElementById('selected-file-name');
        this.elements.writeD64Btn = document.getElementById('write-d64-btn');
        this.elements.d64Filename = document.getElementById('d64-filename');
        this.elements.readD64Btn = document.getElementById('read-d64-btn');

        // Progress elements
        this.elements.progressSection = document.getElementById('progress-section');
        this.elements.progressFill = document.getElementById('progress-fill');
        this.elements.progressText = document.getElementById('progress-text');
        this.elements.progressPercent = document.getElementById('progress-percent');
        this.elements.cancelBtn = document.getElementById('cancel-operation-btn');

        // Status elements
        this.elements.statusLog = document.getElementById('status-log');
        this.elements.clearLogBtn = document.getElementById('clear-log-btn');

        // Debug elements
        this.elements.atnBtn = document.getElementById('atn-btn');
        this.elements.natnBtn = document.getElementById('natn-btn');
    }

    /**
     * Bind event handlers
     */
    bindEvents() {
        this.elements.connectBtn.addEventListener('click', () => this.handleConnect());
        this.elements.disconnectBtn.addEventListener('click', () => this.handleDisconnect());

        this.elements.readDirBtn.addEventListener('click', () => this.handleReadDirectory());
        this.elements.resetDriveBtn.addEventListener('click', () => this.handleResetDrive());

        this.elements.atnBtn.addEventListener('click', () => this.handleATN());
        this.elements.natnBtn.addEventListener('click', () => this.handleNATN());

        this.elements.d64FileInput.addEventListener('change', (e) => this.handleFileSelection(e));
        this.elements.writeD64Btn.addEventListener('click', () => this.handleWriteD64());
        this.elements.readD64Btn.addEventListener('click', () => this.handleReadD64());

        this.elements.cancelBtn.addEventListener('click', () => this.handleCancelOperation());
        this.elements.clearLogBtn.addEventListener('click', () => this.clearLog());
    }

    /**
     * Handle device connection
     */
    async handleConnect() {
        try {
            this.logMessage('Connecting to XUM1541 device...', 'info');
            this.elements.connectBtn.disabled = true;

            this.xum1541 = new XUM1541WebUSB();
            const deviceInfo = await this.xum1541.connect();

            this.cbmProtocol = new CBMProtocol(this.xum1541);
            this.d64Handler = new D64Handler(this.cbmProtocol);
            this.d64Handler.setProgressCallback((percent, text) => this.updateProgress(percent, text));

            this.updateConnectionStatus(true, deviceInfo);
            this.logMessage('Successfully connected to XUM1541!', 'success');

            // Send shutdown command after initialization to prevent device blinking
            try {
                await this.xum1541.shutdownDevice();
            } catch (shutdownError) {
                console.warn('Failed to shutdown device after initialization:', shutdownError);
            }

        } catch (error) {
            this.logMessage(`Connection failed: ${error.message}`, 'error');
            this.updateConnectionStatus(false);
        }

        this.elements.connectBtn.disabled = false;
    }

    /**
     * Handle device disconnection
     */
    async handleDisconnect() {
        try {
            if (this.xum1541) {
                await this.xum1541.disconnect();
            }
            this.updateConnectionStatus(false);
            this.logMessage('Disconnected from XUM1541', 'info');
        } catch (error) {
            this.logMessage(`Disconnect error: ${error.message}`, 'error');
        }

        this.xum1541 = null;
        this.cbmProtocol = null;
        this.d64Handler = null;
    }

    /**
     * Update connection status display
     */
    updateConnectionStatus(connected, deviceInfo = null) {
        if (connected) {
            this.elements.connectionStatus.className = 'status-indicator connected';
            this.elements.connectionText.textContent = 'Connected';
            this.elements.connectBtn.disabled = true;
            this.elements.disconnectBtn.disabled = false;
            this.elements.driveSection.style.display = 'block';

            if (deviceInfo) {
                this.elements.deviceInfo.style.display = 'block';
                this.elements.deviceName.textContent = 'XUM1541';
                this.elements.firmwareVersion.textContent = deviceInfo.version;
                this.elements.deviceCapabilities.textContent = this.xum1541.getCapabilitiesString(deviceInfo.capabilities);
            }
        } else {
            this.elements.connectionStatus.className = 'status-indicator disconnected';
            this.elements.connectionText.textContent = 'Not Connected';
            this.elements.connectBtn.disabled = false;
            this.elements.disconnectBtn.disabled = true;
            this.elements.driveSection.style.display = 'none';
            this.elements.deviceInfo.style.display = 'none';
            this.hideProgress();
        }
    }

    /**
     * Handle directory reading
     */
    async handleReadDirectory() {
        if (!this.cbmProtocol) return;

        const driveAddress = parseInt(this.elements.driveSelect.value);

        try {
            this.elements.readDirBtn.disabled = true;
            this.logMessage(`Reading directory from drive ${driveAddress}...`, 'info');

            const entries = await this.cbmProtocol.readDirectory(driveAddress);
            this.displayDirectory(entries);

            this.logMessage(`Successfully read ${entries.length} directory entries`, 'success');
        } catch (error) {
            this.logMessage(`Failed to read directory: ${error.message}`, 'error');
        } finally {
            // Send shutdown command to prevent device blinking
            if (this.xum1541) {
                try {
                    await this.xum1541.shutdownDevice();
                } catch (shutdownError) {
                    console.warn('Failed to shutdown device:', shutdownError);
                }
            }
        }

        this.elements.readDirBtn.disabled = false;
    }

    /**
     * Display directory entries
     */
    displayDirectory(entries) {
        let html = '<div class="directory-header"><h4>Directory Contents</h4></div>';

        if (entries.length === 0) {
            html += '<div class="directory-empty">Directory is empty</div>';
        } else {
            html += '<table class="directory-table">';
            html += '<thead><tr><th>Blocks</th><th>Filename</th></tr></thead><tbody>';

            entries.forEach(entry => {
                html += `<tr><td>${entry.blocks}</td><td class="filename">${entry.filename}</td></tr>`;
            });

            html += '</tbody></table>';
        }

        this.elements.directoryDisplay.innerHTML = html;
    }

    /**
     * Handle drive reset
     */
    async handleResetDrive() {
        if (!this.xum1541) return;

        try {
            this.elements.resetDriveBtn.disabled = true;
            this.logMessage('Resetting drive...', 'info');

            await this.xum1541.resetBus();

            this.logMessage('Drive reset completed', 'success');
        } catch (error) {
            this.logMessage(`Drive reset failed: ${error.message}`, 'error');
        } finally {
            // Send shutdown command to prevent device blinking
            if (this.xum1541) {
                try {
                    await this.xum1541.shutdownDevice();
                } catch (shutdownError) {
                    console.warn('Failed to shutdown device:', shutdownError);
                }
            }
        }

        this.elements.resetDriveBtn.disabled = false;
    }

    /**
     * Handle file selection for D64 upload
     */
    handleFileSelection(event) {
        const file = event.target.files[0];

        if (file) {
            this.elements.selectedFileName.textContent = file.name;
            this.elements.writeD64Btn.disabled = false;
            this.logMessage(`Selected file: ${file.name} (${file.size} bytes)`, 'info');
        } else {
            this.elements.selectedFileName.textContent = '';
            this.elements.writeD64Btn.disabled = true;
        }
    }

    /**
     * Handle D64 write to drive
     */
    async handleWriteD64() {
        if (!this.d64Handler || !this.elements.d64FileInput.files[0]) return;

        const file = this.elements.d64FileInput.files[0];
        const driveAddress = parseInt(this.elements.driveSelect.value);

        try {
            this.currentOperation = 'write';
            this.showProgress('Uploading D64...');
            this.elements.writeD64Btn.disabled = true;

            // Parse D64 file
            const d64Info = await this.d64Handler.parseD64File(file);
            this.logMessage(`Parsed D64: ${d64Info.tracks} tracks, ${d64Info.size} bytes`, 'info');

            // Write to drive
            await this.d64Handler.writeD64ToDrive(driveAddress, d64Info.data, d64Info.tracks);

            this.logMessage(`Successfully wrote ${file.name} to drive ${driveAddress}`, 'success');
        } catch (error) {
            this.logMessage(`D64 write failed: ${error.message}`, 'error');
        } finally {
            this.currentOperation = null;
            this.hideProgress();
            this.elements.writeD64Btn.disabled = false;

            // Send shutdown command to prevent device blinking
            if (this.xum1541) {
                try {
                    await this.xum1541.shutdownDevice();
                } catch (shutdownError) {
                    console.warn('Failed to shutdown device:', shutdownError);
                }
            }
        }
    }

    /**
     * Handle D64 read from drive
     */
    async handleReadD64() {
        if (!this.d64Handler) return;

        const driveAddress = parseInt(this.elements.driveSelect.value);
        const filename = this.elements.d64Filename.value.trim() || 'DISK';

        try {
            this.currentOperation = 'read';
            this.showProgress('Downloading D64...');
            this.elements.readD64Btn.disabled = true;

            // Read from drive
            const d64Data = await this.d64Handler.readD64FromDrive(driveAddress, 35);

            // Create download
            this.d64Handler.createD64Download(d64Data, filename);

            this.logMessage(`Successfully read D64 from drive ${driveAddress}`, 'success');
        } catch (error) {
            this.logMessage(`D64 read failed: ${error.message}`, 'error');
        } finally {
            this.currentOperation = null;
            this.hideProgress();
            this.elements.readD64Btn.disabled = false;

            // Send shutdown command to prevent device blinking
            if (this.xum1541) {
                try {
                    await this.xum1541.shutdownDevice();
                } catch (shutdownError) {
                    console.warn('Failed to shutdown device:', shutdownError);
                }
            }
        }
    }

    /**
     * Handle operation cancellation
     */
    handleCancelOperation() {
        if (this.currentOperation && this.d64Handler) {
            this.d64Handler.requestCancel();
            this.logMessage('Operation cancelled by user', 'warning');
        }
    }

    /**
     * Handle ATN debug command (pull ATN low)
     */
    async handleATN() {
        if (!this.cbmProtocol) return;

        try {
            this.elements.atnBtn.disabled = true;
            this.logMessage('Pulling ATN low...', 'info');

            await this.cbmProtocol.setATN(true);

            this.logMessage('ATN pulled low successfully', 'success');
        } catch (error) {
            this.logMessage(`ATN command failed: ${error.message}`, 'error');
        } finally {
            // Send shutdown command to prevent device blinking
            if (this.xum1541) {
                try {
                    await this.xum1541.shutdownDevice();
                } catch (shutdownError) {
                    console.warn('Failed to shutdown device:', shutdownError);
                }
            }
        }

        this.elements.atnBtn.disabled = false;
    }

    /**
     * Handle NATN debug command (release ATN)
     */
    async handleNATN() {
        if (!this.cbmProtocol) return;

        try {
            this.elements.natnBtn.disabled = true;
            this.logMessage('Releasing ATN...', 'info');

            await this.cbmProtocol.setATN(false);

            this.logMessage('ATN released successfully', 'success');
        } catch (error) {
            this.logMessage(`NATN command failed: ${error.message}`, 'error');
        } finally {
            // Send shutdown command to prevent device blinking
            if (this.xum1541) {
                try {
                    await this.xum1541.shutdownDevice();
                } catch (shutdownError) {
                    console.warn('Failed to shutdown device:', shutdownError);
                }
            }
        }

        this.elements.natnBtn.disabled = false;
    }

    /**
     * Show progress section
     */
    showProgress(text) {
        this.elements.progressSection.style.display = 'block';
        this.elements.progressText.textContent = text;
        this.elements.progressPercent.textContent = '0%';
        this.elements.progressFill.style.width = '0%';
        this.elements.cancelBtn.style.display = 'inline-block';
    }

    /**
     * Update progress display
     */
    updateProgress(percent, text) {
        this.elements.progressFill.style.width = `${percent}%`;
        this.elements.progressPercent.textContent = `${percent}%`;
        if (text) {
            this.elements.progressText.textContent = text;
        }
    }

    /**
     * Hide progress section
     */
    hideProgress() {
        this.elements.progressSection.style.display = 'none';
        this.elements.cancelBtn.style.display = 'none';
    }

    /**
     * Log a message to the status area
     */
    logMessage(message, type = 'info') {
        const timestamp = new Date().toLocaleTimeString();
        const logEntry = document.createElement('div');
        logEntry.className = `log-entry log-${type}`;
        logEntry.innerHTML = `<span class="log-time">${timestamp}</span> <span class="log-message">${message}</span>`;

        this.elements.statusLog.appendChild(logEntry);
        this.elements.statusLog.scrollTop = this.elements.statusLog.scrollHeight;

        console.log(`[${type.toUpperCase()}] ${message}`);
    }

    /**
     * Clear the log
     */
    clearLog() {
        this.elements.statusLog.innerHTML = '';
    }
}

// Initialize the UI when the page loads
document.addEventListener('DOMContentLoaded', () => {
    // Check WebUSB support
    if (!XUM1541WebUSB.isSupported()) {
        alert('WebUSB is not supported by this browser. Please use Chrome or Edge with HTTPS.');
        return;
    }

    // Initialize the UI controller
    window.uiController = new UIController();
});

// Export for debugging
window.UIController = UIController;