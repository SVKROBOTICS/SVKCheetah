# SVK Robotics Cheetah Line Follow Robot

This is a custom Arduino library for the SVK Cheetah Line Follow Robot that includes a modified version of the IRremote library for remote operation.

## Installation Instructions

1. **Download the ZIP file:**
   - Go to the [SVK Robotics Cheetah repository in github](https://github.com/SVKROBOTICS/SVKCheetah).
   - Press the "Code" button then press "Download ZIP", or you can [**click here**](https://github.com/SVKROBOTICS/SVKCheetah/archive/refs/heads/main.zip).

2. **Remove Existing IRremote Library:**
   - Before importing the new library, ensure that any existing installations of the IRremote library are removed or renamed to prevent conflicts.
   - The Arduino libraries folder is typically located in your sketchbook directory. You can find the location in the Arduino IDE by going to `File > Preferences` and looking for the `Sketchbook location` path.

3. **Import the Library:**
   - Open the Arduino IDE.
   - Go to `Sketch > Include Library > Add .ZIP Library...`.
   - Select the downloaded `SVKCheetah-main.zip` file to import it.

4. **Use the Library:**
   - After importing, you can include `SVKCheetah` in your sketches:
     ```cpp
     #include <SVKCheetah.h>
     ```

## Notes
- Ensure that no other versions of the IRremote library are installed in the Arduino libraries folder, as they may conflict with the modified version provided.
