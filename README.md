The main controller file is:

mavic_controller.py

This is the controller that contains the drone navigation logic and is the code that should be reviewed and connected to the simulation environments.

The three simulation worlds are:

- forest.wbt
- maze.wbt
- university.wbt

I also added links to videos in the OneDrive folder:

videos_wo_and_with_canny

These videos demonstrate the behavior with and without Canny edge detection enabled.

Relevant screenshots:

- canny_danger_20260615_155328.png
- depth_danger_20260615_155328.png

Additional screenshots can be found in the folder:

withCanny

The purpose of these files is to compare the drone's behavior when using only depth information versus depth information combined with Canny edge detection.
