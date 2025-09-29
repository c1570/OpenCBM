/**
 * XUM1541 WebUSB Communication Layer
 *
 * This module handles low-level USB communication with XUM1541 devices
 * using the WebUSB API. It implements the XUM1541 protocol as defined
 * in the firmware.
 */

class XUM1541WebUSB {
    constructor() {
        this.device = null;
        this.isConnected = false;
        this.interfaceNumber = 0;
        this.bulkInEndpoint = 3;
        this.bulkOutEndpoint = 4;

        // XUM1541 USB identifiers (from xum1541_types.h)
        this.vendorId = 0x16d0;
        this.productId = 0x0504;

        // XUM1541 protocol constants
        this.XUM1541_ECHO = 0;
        this.XUM1541_INIT = 1;
        this.XUM1541_RESET = 2;
        this.XUM1541_SHUTDOWN = 3;
        this.XUM1541_ENTER_BOOTLOADER = 4;
        this.XUM1541_READ = 8;
        this.XUM1541_WRITE = 9;

        // Ioctl commands
        this.XUM1541_IOCTL = 16;
        this.XUM1541_IEC_SETRELEASE = this.XUM1541_IOCTL + 13;

        // Protocol flags
        this.XUM1541_CBM = (1 << 4);
        this.XUM_WRITE_TALK = (1 << 0);
        this.XUM_WRITE_ATN = (1 << 1);

        // Status codes
        this.XUM1541_IO_BUSY = 1;
        this.XUM1541_IO_READY = 2;
        this.XUM1541_IO_ERROR = 3;

        // Capabilities
        this.XUM1541_CAP_CBM = 0x01;
        this.XUM1541_CAP_NIB = 0x02;
        this.XUM1541_CAP_NIB_SRQ = 0x04;
        this.XUM1541_CAP_IEEE488 = 0x08;
        this.XUM1541_CAP_TAP = 0x10;
    }

    /**
     * Check if WebUSB is supported by the browser
     */
    static isSupported() {
        return 'usb' in navigator;
    }

    /**
     * Request and connect to XUM1541 device
     */
    async connect() {
        if (!XUM1541WebUSB.isSupported()) {
            throw new Error('WebUSB is not supported by this browser. Use Chrome/Edge with HTTPS.');
        }

        try {
            // Request device
            this.device = await navigator.usb.requestDevice({
                filters: [{
                    vendorId: this.vendorId,
                    productId: this.productId
                }]
            });

            console.log('XUM1541 device selected:', this.device);

            // Open device
            await this.device.open();

            // Select configuration (assuming configuration 1)
            if (this.device.configuration === null) {
                await this.device.selectConfiguration(1);
            }

            // Claim interface
            await this.device.claimInterface(this.interfaceNumber);

            this.isConnected = true;
            console.log('XUM1541 device connected successfully');

            // Initialize the device
            const initResult = await this.initDevice();
            console.log('Device initialization result:', initResult);

            return initResult;

        } catch (error) {
            console.error('Failed to connect to XUM1541 device:', error);
            this.isConnected = false;
            throw error;
        }
    }

    /**
     * Disconnect from the device
     */
    async disconnect() {
        if (this.device && this.isConnected) {
            try {
                // Send shutdown command
                await this.controlTransfer(this.XUM1541_SHUTDOWN);

                // Release interface and close device
                await this.device.releaseInterface(this.interfaceNumber);
                await this.device.close();
            } catch (error) {
                console.warn('Error during disconnect:', error);
            }
        }

        this.device = null;
        this.isConnected = false;
        console.log('XUM1541 device disconnected');
    }

    /**
     * Send control transfer command
     */
    async controlTransfer(command, data = null, expectResponse = false) {
        if (!this.isConnected || !this.device) {
            throw new Error('Device not connected');
        }

        const setup = {
            requestType: 'vendor',
            recipient: 'device',
            request: command,
            value: 0,
            index: 0
        };

        let result;

        if (data) {
            // OUT control transfer
            result = await this.device.controlTransferOut(setup, data);
        } else if (expectResponse) {
            // IN control transfer
            result = await this.device.controlTransferIn(setup, 64); // Max response size
        } else {
            // OUT control transfer with no data
            result = await this.device.controlTransferOut(setup);
        }

        return result;
    }

    /**
     * Send bulk data to device
     */
    async bulkOut(data) {
        if (!this.isConnected || !this.device) {
            throw new Error('Device not connected');
        }

        try {
            const result = await this.device.transferOut(this.bulkOutEndpoint, data);
            return result;
        } catch (error) {
            console.error('Bulk OUT transfer failed:', error);
            throw error;
        }
    }

    /**
     * Receive bulk data from device
     */
    async bulkIn(length) {
        if (!this.isConnected || !this.device) {
            throw new Error('Device not connected');
        }

        try {
            const result = await this.device.transferIn(this.bulkInEndpoint, length);
            return new Uint8Array(result.data.buffer);
        } catch (error) {
            console.error('Bulk IN transfer failed:', error);
            throw error;
        }
    }

    /**
     * Initialize XUM1541 device
     */
    async initDevice() {
        console.log('Initializing XUM1541 device...');

        try {
            const result = await this.controlTransfer(this.XUM1541_INIT, null, true);

            if (result.data && result.data.byteLength >= 8) {
                const data = new Uint8Array(result.data.buffer);

                const deviceInfo = {
                    version: data[0],
                    capabilities: data[1],
                    status: data[2],
                    // Additional bytes are reserved
                };

                console.log('Device info:', deviceInfo);
                return deviceInfo;
            } else {
                throw new Error('Invalid response from XUM1541_INIT');
            }
        } catch (error) {
            console.error('Device initialization failed:', error);
            throw error;
        }
    }

    /**
     * Reset CBM bus/drive
     */
    async resetBus() {
        console.log('Resetting CBM bus...');
        await this.controlTransfer(this.XUM1541_RESET);
    }

    /**
     * Send shutdown command to device (prevents blinking)
     */
    async shutdownDevice() {
        console.log('Sending shutdown command to prevent device blinking...');
        await this.controlTransfer(this.XUM1541_SHUTDOWN);
    }

    /**
     * Set/release IEC lines (for ATN control)
     */
    async iecSetRelease(pullLow) {
        console.log(`IEC Set/Release: ${pullLow ? 'SET (pull low)' : 'RELEASE (let high)'}`);

        // Ioctl commands are sent as bulk transfers with 4-byte command header
        // Based on firmware: cmds->cbm_setrelease(/*set*/request[1], /*release*/request[2]);
        // ATN bit is 0x04 (IEC_ATN)
        const command = new Uint8Array(4);
        command[0] = this.XUM1541_IEC_SETRELEASE;  // Command
        if (pullLow) {
            command[1] = 0x04; // set ATN (IEC_ATN = 0x04)
            command[2] = 0x00; // don't release anything
        } else {
            command[1] = 0x00; // don't set anything
            command[2] = 0x04; // release ATN (IEC_ATN = 0x04)
        }
        command[3] = 0; // length high byte (not used for ioctl)

        // Send ioctl command as bulk transfer
        await this.bulkOut(command);

        // Read status response
        const status = await this.bulkIn(3);
        return this.parseStatus(status);
    }

    /**
     * Send CBM command with data
     */
    async sendCBMCommand(data, flags = 0) {
        console.log('Sending CBM command, length:', data.length, 'flags:', flags);

        // Create 4-byte command header
        const command = new Uint8Array(4);
        command[0] = this.XUM1541_WRITE;
        command[1] = this.XUM1541_CBM | flags;
        command[2] = data.length & 0xFF;        // Length low byte
        command[3] = (data.length >> 8) & 0xFF; // Length high byte

        // Send command header
        await this.bulkOut(command);

        // Send data if present
        if (data.length > 0) {
            await this.bulkOut(data);
        }

        // Read status response
        const status = await this.bulkIn(3);
        return this.parseStatus(status);
    }

    /**
     * Read CBM data
     */
    async readCBMData(length, flags = 0) {
        console.log('Reading CBM data, length:', length, 'flags:', flags);

        // Create 4-byte command header
        const command = new Uint8Array(4);
        command[0] = this.XUM1541_READ;
        command[1] = this.XUM1541_CBM | flags;
        command[2] = length & 0xFF;        // Length low byte
        command[3] = (length >> 8) & 0xFF; // Length high byte

        // Send command header
        await this.bulkOut(command);

        // Read data
        const data = await this.bulkIn(length);

        // Read status response
        const status = await this.bulkIn(3);
        const statusInfo = this.parseStatus(status);

        return {
            data: data,
            status: statusInfo
        };
    }

    /**
     * Parse 3-byte status response
     */
    parseStatus(statusBytes) {
        const status = statusBytes[0];
        const value = statusBytes[1] | (statusBytes[2] << 8);

        return {
            code: status,
            value: value,
            ready: status === this.XUM1541_IO_READY,
            busy: status === this.XUM1541_IO_BUSY,
            error: status === this.XUM1541_IO_ERROR
        };
    }

    /**
     * Get capability description string
     */
    getCapabilitiesString(capabilities) {
        const caps = [];

        if (capabilities & this.XUM1541_CAP_CBM) caps.push('CBM');
        if (capabilities & this.XUM1541_CAP_NIB) caps.push('Nibbler');
        if (capabilities & this.XUM1541_CAP_NIB_SRQ) caps.push('SRQ Nibbler');
        if (capabilities & this.XUM1541_CAP_IEEE488) caps.push('IEEE-488');
        if (capabilities & this.XUM1541_CAP_TAP) caps.push('Tape');

        return caps.length > 0 ? caps.join(', ') : 'None';
    }
}

// Export for use by other modules
window.XUM1541WebUSB = XUM1541WebUSB;