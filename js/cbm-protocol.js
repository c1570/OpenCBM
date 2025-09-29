/**
 * CBM Protocol Layer
 *
 * This module implements the CBM IEC protocol commands on top of
 * the XUM1541 USB layer. It provides higher-level functions for
 * common CBM operations like LISTEN, TALK, and device commands.
 */

class CBMProtocol {
    constructor(xum1541Device) {
        this.device = xum1541Device;
        this.lastStatus = null;
    }

    /**
     * Convert ASCII string to PETSCII
     */
    asciiToPetscii(str) {
        const petsciiBytes = [];
        for (let i = 0; i < str.length; i++) {
            let char = str.charCodeAt(i);

            // Basic ASCII to PETSCII conversion
            if (char >= 65 && char <= 90) {
                // A-Z -> a-z in PETSCII
                petsciiBytes.push(char + 32);
            } else if (char >= 97 && char <= 122) {
                // a-z -> A-Z in PETSCII
                petsciiBytes.push(char - 32);
            } else {
                // Other characters remain the same
                petsciiBytes.push(char);
            }
        }
        return new Uint8Array(petsciiBytes);
    }

    /**
     * Convert PETSCII bytes to ASCII string
     */
    petsciiToAscii(bytes) {
        let result = '';
        for (let i = 0; i < bytes.length; i++) {
            let char = bytes[i];

            // Basic PETSCII to ASCII conversion
            if (char >= 65 && char <= 90) {
                // A-Z -> a-z in ASCII
                result += String.fromCharCode(char + 32);
            } else if (char >= 97 && char <= 122) {
                // a-z -> A-Z in ASCII
                result += String.fromCharCode(char - 32);
            } else if (char >= 32 && char <= 126) {
                // Printable ASCII characters
                result += String.fromCharCode(char);
            } else if (char === 0) {
                // Null terminator
                break;
            } else {
                // Non-printable or special characters
                result += '?';
            }
        }
        return result;
    }

    /**
     * Send LISTEN command to device
     */
    async sendListen(deviceAddress, secondaryAddress = null) {
        console.log(`LISTEN ${deviceAddress}${secondaryAddress !== null ? ',' + secondaryAddress : ''}`);

        const commands = [];

        // LISTEN command (0x20 + device address)
        commands.push(0x20 + deviceAddress);

        // Secondary address if provided (0x60 + secondary address)
        if (secondaryAddress !== null) {
            commands.push(0x60 + secondaryAddress);
        }

        const result = await this.device.sendCBMCommand(new Uint8Array(commands), this.device.XUM_WRITE_ATN);
        this.lastStatus = result;
        return result;
    }

    /**
     * Send TALK command to device
     */
    async sendTalk(deviceAddress, secondaryAddress = null) {
        console.log(`TALK ${deviceAddress}${secondaryAddress !== null ? ',' + secondaryAddress : ''}`);

        const commands = [];

        // TALK command (0x40 + device address)
        commands.push(0x40 + deviceAddress);

        // Secondary address if provided (0x60 + secondary address)
        if (secondaryAddress !== null) {
            commands.push(0x60 + secondaryAddress);
        }

        const result = await this.device.sendCBMCommand(new Uint8Array(commands), this.device.XUM_WRITE_ATN);
        this.lastStatus = result;
        return result;
    }

    /**
     * Send UNTALK command
     */
    async sendUntalk() {
        console.log('UNTALK');
        const result = await this.device.sendCBMCommand(new Uint8Array([0x5F]), this.device.XUM_WRITE_ATN);
        this.lastStatus = result;
        return result;
    }

    /**
     * Send UNLISTEN command
     */
    async sendUnlisten() {
        console.log('UNLISTEN');
        const result = await this.device.sendCBMCommand(new Uint8Array([0x3F]), this.device.XUM_WRITE_ATN);
        this.lastStatus = result;
        return result;
    }

    /**
     * Open a file on the CBM device
     */
    async openFile(deviceAddress, secondaryAddress, filename) {
        console.log(`Opening file "${filename}" on device ${deviceAddress}, secondary ${secondaryAddress}`);

        // Convert filename to PETSCII
        const petsciiFilename = this.asciiToPetscii(filename);

        // LISTEN + secondary address
        await this.sendListen(deviceAddress, secondaryAddress);

        // Send filename data
        await this.device.sendCBMCommand(petsciiFilename);

        // UNLISTEN
        await this.sendUnlisten();
    }

    /**
     * Close a file on the CBM device
     */
    async closeFile(deviceAddress, secondaryAddress) {
        console.log(`Closing file on device ${deviceAddress}, secondary ${secondaryAddress}`);

        // LISTEN + secondary address + CLOSE command
        await this.sendListen(deviceAddress, secondaryAddress);
        await this.sendUnlisten();
    }

    /**
     * Read data from an open file
     */
    async readFileData(deviceAddress, secondaryAddress, maxLength = 1024) {
        console.log(`Reading data from device ${deviceAddress}, secondary ${secondaryAddress}`);

        // TALK + secondary address
        await this.sendTalk(deviceAddress, secondaryAddress);

        // Read data
        const result = await this.device.readCBMData(maxLength);

        // UNTALK
        await this.sendUntalk();

        return result;
    }

    /**
     * Write data to an open file
     */
    async writeFileData(deviceAddress, secondaryAddress, data) {
        console.log(`Writing ${data.length} bytes to device ${deviceAddress}, secondary ${secondaryAddress}`);

        // LISTEN + secondary address
        await this.sendListen(deviceAddress, secondaryAddress);

        // Send data
        await this.device.sendCBMCommand(data);

        // UNLISTEN
        await this.sendUnlisten();
    }

    /**
     * Get device status (error channel)
     */
    async getDeviceStatus(deviceAddress) {
        console.log(`Getting status from device ${deviceAddress}`);

        try {
            // Open command channel (secondary address 15)
            await this.sendTalk(deviceAddress, 15);

            // Read status
            const result = await this.device.readCBMData(256);

            // UNTALK
            await this.sendUntalk();

            const statusText = this.petsciiToAscii(result.data);
            console.log(`Device ${deviceAddress} status: ${statusText}`);

            return statusText;
        } catch (error) {
            console.error('Failed to get device status:', error);
            return 'ERROR: Could not read status';
        }
    }

    /**
     * Send a command to the device command channel
     */
    async sendDeviceCommand(deviceAddress, command) {
        console.log(`Sending command "${command}" to device ${deviceAddress}`);

        // Convert command to PETSCII
        const petsciiCommand = this.asciiToPetscii(command);

        // LISTEN to command channel (secondary address 15)
        await this.sendListen(deviceAddress, 15);

        // Send command
        await this.device.sendCBMCommand(petsciiCommand);

        // UNLISTEN
        await this.sendUnlisten();

        // Get status response
        return await this.getDeviceStatus(deviceAddress);
    }

    /**
     * Read directory from device
     */
    async readDirectory(deviceAddress, pattern = '*') {
        console.log(`Reading directory from device ${deviceAddress} with pattern "${pattern}"`);

        const entries = [];

        try {
            // Open directory (filename is "$" optionally followed by pattern)
            const dirCommand = pattern === '*' ? '$' : `$${pattern}`;
            await this.openFile(deviceAddress, 0, dirCommand);

            // Read directory data
            await this.sendTalk(deviceAddress, 0);

            // Read first two bytes (link to next directory entry - not used)
            let result = await this.device.readCBMData(2);
            if (result.data.length < 2) {
                throw new Error('Failed to read directory header');
            }

            // Read directory entries
            while (true) {
                // Read next entry link (2 bytes)
                result = await this.device.readCBMData(2);
                if (result.data.length < 2) break;

                const link = result.data[0] | (result.data[1] << 8);
                if (link === 0) break; // End of directory

                // Read file size (2 bytes)
                result = await this.device.readCBMData(2);
                if (result.data.length < 2) break;

                const blocks = result.data[0] | (result.data[1] << 8);

                // Read filename (until null terminator or EOL)
                const filenameBytes = [];
                while (true) {
                    result = await this.device.readCBMData(1);
                    if (result.data.length === 0) break;

                    const byte = result.data[0];
                    if (byte === 0) break; // Null terminator

                    filenameBytes.push(byte);

                    // Safety check for runaway reads
                    if (filenameBytes.length > 50) break;
                }

                if (filenameBytes.length > 0) {
                    const filename = this.petsciiToAscii(new Uint8Array(filenameBytes));
                    entries.push({
                        filename: filename.trim(),
                        blocks: blocks
                    });
                }
            }

            // UNTALK
            await this.sendUntalk();

            // Close directory
            await this.closeFile(deviceAddress, 0);

        } catch (error) {
            console.error('Failed to read directory:', error);
            // Try to clean up
            try {
                await this.sendUntalk();
                await this.closeFile(deviceAddress, 0);
            } catch (cleanupError) {
                console.warn('Cleanup error:', cleanupError);
            }
            throw error;
        }

        console.log(`Read ${entries.length} directory entries`);
        return entries;
    }

    /**
     * Format a disk
     */
    async formatDisk(deviceAddress, diskName = 'FORMATTED', diskId = '01') {
        console.log(`Formatting disk on device ${deviceAddress} with name "${diskName}", ID "${diskId}"`);

        const formatCommand = `NEW:${diskName},${diskId}`;
        return await this.sendDeviceCommand(deviceAddress, formatCommand);
    }

    /**
     * Validate a disk (like VALIDATE command)
     */
    async validateDisk(deviceAddress) {
        console.log(`Validating disk on device ${deviceAddress}`);

        return await this.sendDeviceCommand(deviceAddress, 'VALIDATE');
    }

    /**
     * Initialize disk drive
     */
    async initializeDrive(deviceAddress) {
        console.log(`Initializing drive ${deviceAddress}`);

        return await this.sendDeviceCommand(deviceAddress, 'INITIALIZE');
    }

    /**
     * Set ATN line state (for debugging)
     */
    async setATN(pullLow) {
        console.log(`Setting ATN line: ${pullLow ? 'LOW (active)' : 'HIGH (released)'}`);

        try {
            const result = await this.device.iecSetRelease(pullLow);
            return result;
        } catch (error) {
            console.error('Failed to set ATN line:', error);
            throw error;
        }
    }
}

// Export for use by other modules
window.CBMProtocol = CBMProtocol;