#!/usr/bin/env python
# xyw lidar test cfg
PACKAGE = "lv_align_verify"
from dynamic_reconfigure.parameter_generator_catkin import *
gen = ParameterGenerator()

gen.add("margin", int_t, 0, "The margin of the pixel of points in image", 2, 0, 10)
gen.add("transform_c_l_x", double_t, 0, "c_l_x", 0) 
gen.add("str_param", str_t, 0, "A String parameter", "Hello World")
gen.add("bool_param", bool_t, 0, "A Boolean parameter", True)

size_enum = gen.enum([gen.const("Small", int_t, 0, "A small constant"),
                      gen.const("Medium", int_t, 1, "A medium constant"),
                      gen.const("Large", int_t, 2, "A large constant"),
                      gen.const("Extralarge", int_t, 3, "A extra large constant")],
                      "An enum to set size")
gen.add("size", int_t, 0, "A size parameter which is edited via an enum", 1, 0, 3, edit_method=size_enum)
gen.add("mark_threshold", int_t, 0, 'The maximum number of marked cells allowed in a column considered to be free', 2, 0, 16)
exit(gen.generate("xyw_lidar_test", "xyw_lidar_test_node", "XYWLidarTest"))
# the second param 'node_name' is not limited.