zephyr_library()

zephyr_library_sources_ifdef(
  CONFIG_ZMK_INPUT_PROCESSOR_AXIS_BEHAVIORS
  src/input_processor_axis_behaviors.c
)
