# SVK Robotics Cheetah Line Follow Robot

This is a custom Arduino library that includes a modified version of the IRremote library.

## Installation Instructions

1. **Download the ZIP file:**
   - Download the `MyLibrary.zip` file.

2. **Remove Existing IRremote Library:**
   - Before importing the new library, ensure that any existing installations of the IRremote library are removed or renamed to prevent conflicts.
   - The Arduino libraries folder is typically located in your sketchbook directory. You can find the location in the Arduino IDE by going to `File > Preferences` and looking for the `Sketchbook location` path.

3. **Import the Library:**
   - Open the Arduino IDE.
   - Go to `Sketch > Include Library > Add .ZIP Library...`.
   - Select the downloaded `MyLibrary.zip` file to import it.

4. **Use the Library:**
   - After importing, you can include `MyLibrary` in your sketches:
     ```cpp
     #include <MyLibrary.h>
     ```

## Notes
- Ensure that no other versions of the IRremote library are installed in the Arduino libraries folder, as they may conflict with the modified version provided.
