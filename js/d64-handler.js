/**
 * D64 Disk Image Handler
 *
 * This module handles D64 disk image format operations including
 * reading from drives, writing to drives, and file format parsing.
 */

class D64Handler {
    constructor(cbmProtocol) {
        this.cbm = cbmProtocol;

        // D64 format constants
        this.TRACKS_35 = 35;
        this.TRACKS_40 = 40; // Extended D64 format
        this.SECTORS_PER_TRACK = [
            0,  // Track 0 (not used)
            21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, // Tracks 1-17
            19, 19, 19, 19, 19, 19, 19,                                         // Tracks 18-24
            18, 18, 18, 18, 18, 18,                                             // Tracks 25-30
            17, 17, 17, 17, 17,                                                 // Tracks 31-35
            17, 17, 17, 17, 17                                                  // Tracks 36-40 (extended)
        ];
        this.BYTES_PER_SECTOR = 256;
        this.STANDARD_D64_SIZE = 174848; // 35 tracks
        this.EXTENDED_D64_SIZE = 196608; // 40 tracks

        this.progressCallback = null;
        this.cancelRequested = false;
    }

    /**
     * Set progress callback function
     */
    setProgressCallback(callback) {
        this.progressCallback = callback;
    }

    /**
     * Request cancellation of current operation
     */
    requestCancel() {
        this.cancelRequested = true;
    }

    /**
     * Update progress if callback is set
     */
    updateProgress(track, sector, total, operation) {
        if (this.progressCallback) {
            const percent = Math.round((track * 100) / total);
            this.progressCallback(percent, `${operation} Track ${track}, Sector ${sector}`);
        }
    }

    /**
     * Calculate total sectors in a D64 image
     */
    getTotalSectors(tracks = 35) {
        let total = 0;
        for (let track = 1; track <= tracks; track++) {
            total += this.SECTORS_PER_TRACK[track];
        }
        return total;
    }

    /**
     * Convert track/sector to absolute sector number
     */
    trackSectorToAbsolute(track, sector) {
        let absolute = 0;
        for (let t = 1; t < track; t++) {
            absolute += this.SECTORS_PER_TRACK[t];
        }
        return absolute + sector;
    }

    /**
     * Convert absolute sector to track/sector
     */
    absoluteToTrackSector(absolute) {
        let track = 1;
        let remaining = absolute;

        while (track <= 40 && remaining >= this.SECTORS_PER_TRACK[track]) {
            remaining -= this.SECTORS_PER_TRACK[track];
            track++;
        }

        return { track, sector: remaining };
    }

    /**
     * Read a single sector from the drive using memory read
     */
    async readSector(deviceAddress, track, sector) {
        try {
            // Use direct memory read command
            // Memory read: M-R<low addr><high addr><byte count>
            const trackSectorAddress = this.trackSectorToMemoryAddress(track, sector);
            const lowAddr = trackSectorAddress & 0xFF;
            const highAddr = (trackSectorAddress >> 8) & 0xFF;

            const command = `M-R${String.fromCharCode(lowAddr)}${String.fromCharCode(highAddr)}${String.fromCharCode(0)}`;

            await this.cbm.sendDeviceCommand(deviceAddress, command);

            // Read the data from the buffer
            const result = await this.cbm.readFileData(deviceAddress, 0, this.BYTES_PER_SECTOR);

            return result.data;
        } catch (error) {
            console.error(`Failed to read sector ${track}/${sector}:`, error);
            throw error;
        }
    }

    /**
     * Write a single sector to the drive using memory write
     */
    async writeSector(deviceAddress, track, sector, data) {
        try {
            // Use direct memory write command
            // Memory write: M-W<low addr><high addr><byte count><data>
            const trackSectorAddress = this.trackSectorToMemoryAddress(track, sector);
            const lowAddr = trackSectorAddress & 0xFF;
            const highAddr = (trackSectorAddress >> 8) & 0xFF;

            // Build command with data
            const commandBytes = [
                77, 45, 87, // "M-W"
                lowAddr,
                highAddr,
                data.length & 0xFF
            ];

            // Add data bytes
            for (let i = 0; i < data.length; i++) {
                commandBytes.push(data[i]);
            }

            await this.cbm.sendDeviceCommand(deviceAddress, String.fromCharCode(...commandBytes));
        } catch (error) {
            console.error(`Failed to write sector ${track}/${sector}:`, error);
            throw error;
        }
    }

    /**
     * Convert track/sector to memory address (simplified)
     */
    trackSectorToMemoryAddress(track, sector) {
        // This is a simplified conversion - real 1541 memory mapping is more complex
        // For demonstration purposes, we use a basic formula
        return 0x0400 + (track - 1) * 21 + sector;
    }

    /**
     * Read complete D64 image from drive
     */
    async readD64FromDrive(deviceAddress, tracks = 35) {
        console.log(`Reading D64 from drive ${deviceAddress}, ${tracks} tracks`);

        this.cancelRequested = false;
        const d64Data = new Uint8Array(tracks === 35 ? this.STANDARD_D64_SIZE : this.EXTENDED_D64_SIZE);
        let dataOffset = 0;

        try {
            for (let track = 1; track <= tracks; track++) {
                if (this.cancelRequested) {
                    throw new Error('Operation cancelled by user');
                }

                const sectorsInTrack = this.SECTORS_PER_TRACK[track];

                for (let sector = 0; sector < sectorsInTrack; sector++) {
                    this.updateProgress(track, sector, tracks, 'Reading');

                    const sectorData = await this.readSector(deviceAddress, track, sector);

                    // Copy sector data to D64 image
                    const bytesToCopy = Math.min(sectorData.length, this.BYTES_PER_SECTOR);
                    d64Data.set(sectorData.slice(0, bytesToCopy), dataOffset);
                    dataOffset += this.BYTES_PER_SECTOR;
                }
            }

            console.log('D64 read completed successfully');
            return d64Data;
        } catch (error) {
            console.error('Failed to read D64:', error);
            throw error;
        }
    }

    /**
     * Write complete D64 image to drive
     */
    async writeD64ToDrive(deviceAddress, d64Data, tracks = null) {
        console.log(`Writing D64 to drive ${deviceAddress}`);

        this.cancelRequested = false;

        // Auto-detect tracks if not specified
        if (!tracks) {
            tracks = d64Data.length <= this.STANDARD_D64_SIZE ? 35 : 40;
        }

        try {
            // First, format the disk
            this.updateProgress(0, 0, tracks, 'Formatting');
            await this.cbm.formatDisk(deviceAddress, 'UPLOADED', '01');

            // Wait for format to complete (this is a simplified approach)
            await this.sleep(5000);

            let dataOffset = 0;

            for (let track = 1; track <= tracks; track++) {
                if (this.cancelRequested) {
                    throw new Error('Operation cancelled by user');
                }

                const sectorsInTrack = this.SECTORS_PER_TRACK[track];

                for (let sector = 0; sector < sectorsInTrack; sector++) {
                    this.updateProgress(track, sector, tracks, 'Writing');

                    // Extract sector data from D64 image
                    const sectorData = d64Data.slice(dataOffset, dataOffset + this.BYTES_PER_SECTOR);

                    await this.writeSector(deviceAddress, track, sector, sectorData);
                    dataOffset += this.BYTES_PER_SECTOR;
                }
            }

            console.log('D64 write completed successfully');
        } catch (error) {
            console.error('Failed to write D64:', error);
            throw error;
        }
    }

    /**
     * Parse D64 file from File API
     */
    async parseD64File(file) {
        return new Promise((resolve, reject) => {
            const reader = new FileReader();

            reader.onload = (event) => {
                try {
                    const arrayBuffer = event.target.result;
                    const d64Data = new Uint8Array(arrayBuffer);

                    // Validate file size
                    if (d64Data.length !== this.STANDARD_D64_SIZE &&
                        d64Data.length !== this.EXTENDED_D64_SIZE) {
                        reject(new Error(`Invalid D64 file size: ${d64Data.length} bytes. Expected ${this.STANDARD_D64_SIZE} or ${this.EXTENDED_D64_SIZE} bytes.`));
                        return;
                    }

                    const info = {
                        data: d64Data,
                        tracks: d64Data.length === this.STANDARD_D64_SIZE ? 35 : 40,
                        size: d64Data.length,
                        filename: file.name
                    };

                    resolve(info);
                } catch (error) {
                    reject(error);
                }
            };

            reader.onerror = () => {
                reject(new Error('Failed to read file'));
            };

            reader.readAsArrayBuffer(file);
        });
    }

    /**
     * Create downloadable D64 file
     */
    createD64Download(d64Data, filename) {
        const blob = new Blob([d64Data], { type: 'application/octet-stream' });
        const url = URL.createObjectURL(blob);

        const link = document.createElement('a');
        link.href = url;
        link.download = filename.endsWith('.d64') ? filename : filename + '.d64';

        // Trigger download
        document.body.appendChild(link);
        link.click();
        document.body.removeChild(link);

        // Clean up
        setTimeout(() => URL.revokeObjectURL(url), 1000);
    }

    /**
     * Parse directory from D64 data (for validation/info)
     */
    parseD64Directory(d64Data) {
        const entries = [];

        try {
            // Directory is on track 18, starting at sector 1
            const dirTrack = 18;
            let dirSector = 1;

            // Calculate offset to track 18 sector 1
            let offset = 0;
            for (let track = 1; track < dirTrack; track++) {
                offset += this.SECTORS_PER_TRACK[track] * this.BYTES_PER_SECTOR;
            }
            offset += dirSector * this.BYTES_PER_SECTOR;

            // Read directory entries
            while (dirSector !== 0) {
                const sectorData = d64Data.slice(offset, offset + this.BYTES_PER_SECTOR);

                // First two bytes are link to next directory sector
                const nextTrack = sectorData[0];
                const nextSector = sectorData[1];

                // Process directory entries in this sector (8 entries per sector)
                for (let entryIndex = 0; entryIndex < 8; entryIndex++) {
                    const entryOffset = 2 + entryIndex * 32;

                    // Check if entry is valid (file type != 0)
                    if (sectorData[entryOffset] !== 0) {
                        const fileType = sectorData[entryOffset] & 0x0F;
                        const closed = (sectorData[entryOffset] & 0x80) !== 0;

                        // Extract filename (16 bytes, PETSCII)
                        const filenameBytes = sectorData.slice(entryOffset + 3, entryOffset + 19);
                        const filename = this.cbm.petsciiToAscii(filenameBytes).replace(/\0/g, '').trim();

                        // Extract file size in blocks
                        const blocks = sectorData[entryOffset + 28] | (sectorData[entryOffset + 29] << 8);

                        if (filename.length > 0) {
                            entries.push({
                                filename,
                                fileType: this.getFileTypeString(fileType),
                                closed,
                                blocks
                            });
                        }
                    }
                }

                // Move to next directory sector
                if (nextTrack === 0) break;

                // Calculate offset for next sector
                offset = 0;
                for (let track = 1; track < nextTrack; track++) {
                    offset += this.SECTORS_PER_TRACK[track] * this.BYTES_PER_SECTOR;
                }
                offset += nextSector * this.BYTES_PER_SECTOR;

                dirSector = nextSector;
            }
        } catch (error) {
            console.error('Error parsing D64 directory:', error);
        }

        return entries;
    }

    /**
     * Get file type string from type code
     */
    getFileTypeString(fileType) {
        const types = ['DEL', 'SEQ', 'PRG', 'USR', 'REL'];
        return types[fileType] || 'UNK';
    }

    /**
     * Sleep utility function
     */
    sleep(ms) {
        return new Promise(resolve => setTimeout(resolve, ms));
    }
}

// Export for use by other modules
window.D64Handler = D64Handler;