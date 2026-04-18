#!/usr/bin/env python3
import os
import sys
import time

def test_speaker():
    """
    Tests the Jetson Nano DAC/Speaker array by invoking the native ALSA diagnostic tool.
    This bypasses any Python-level dependencies and tests directly at the silicon level.
    """
    print("====================================")
    print("🔈 Jetson Standalone Speaker Diagnostic")
    print("====================================")
    
    print("\nAttempting to lock the /dev/snd ALSA subsystem...")
    print("You should hear an audible 'Front, Center' test sequence.")
    print("Press CTRL+C at any time to abort.\n")
    
    time.sleep(2)
    
    try:
        # speaker-test is natively built into Ubuntu/Jetpack
        # -t wav uses a human voice instead of harsh pink noise
        # -c 2 tests stereo (if available), fallback to mono automatically
        # -l 1 means run the loop exactly 1 time
        exit_code = os.system("speaker-test -t wav -c 2 -l 1")
        
        if exit_code == 0:
            print("\n✅ ALSA Diagnostic passed cleanly. Hardware speaker is operational!")
        else:
            print(f"\n❌ ALSA Diagnostic failed with exit code {exit_code}.")
            print("Ensure the USB DAC or I2S amplifier is physically plugged in.")
            
    except KeyboardInterrupt:
        print("\nDiagnostic aborted by user.")
        sys.exit(0)

if __name__ == '__main__':
    test_speaker()
