## Usage

### Build docker image

Run command

`dts devel build -H <duckiebot_name>.local -f`

Replace `<duckiebot_name>` with the name of your duckiebot.

### Run the container

Run command

`dts devel run -H <duckiebot_name>.local`

Replace `<duckiebot_name>` with the name of your duckiebot.

### Check the result

Run command 

`dts start_gui_tools <duckiebot_name>`

Replace `<duckiebot_name>` with the name of your duckiebot.

Then run command

`rqt_image_view`

and select topic */<robot_name>/at_detect/image/compressed*. The *<robot_name>* is the actual name of your duckiebot.
