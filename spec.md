Naming:
- Subsystems: End with "Subsystem"
- Commands: End with "Command"
- Constants: UPPERCASE_WITH_UNDERSCORES
- Methods/Functions: startWithLowerCaseThenCamelCase
- Classes: CamelCase

Structure:
- Basics: Holds two drivebase subsystems, one for differential drive and one for swerve drive. Extend either of these to add custom functionality or declare your drivebase as an instance.
- Constants: Holds Constants files for both drivebases, as well as other things. Each drivebase has a constants file, and all other constants are stored in LibConsts file
- Utils: Hold useful classes such as camera wrappers, config methods for hardware, math and simulation tools. Usually should be left alone when using ValhallaLib

Ideals:
- Assume as little as possible about the specs of the end user, make implementation possible with as little as possible
    - However, allow for use to the max if need be, provide useful and clean ways of doing so
- Allow for extension by user
- Prevent opacity in how things are done
- do the copying so that the user doesnt have to

Todo:
-Camera
-Testing
-auto + joystick chooser(dashboard in gen.)
-simulation
