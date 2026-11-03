set here [file dirname [file normalize [info script]]]
cd $here
set constraints [open "$here/robot.cst" r]
set constraint_text [read $constraints]
close $constraints
if {![regexp {IO_PORT\s+"ir_rx"[^;]*IO_TYPE=LVCMOS[0-9]+[^;]*BANK_VCCIO=[0-9.]+} $constraint_text]} {
    error "IR receiver E3 uses VCCADJ: confirm P20 voltage and set ir_rx IO_TYPE/BANK_VCCIO in robot.cst before building."
}
open_project "$here/robot.gprj"
set_option -top_module robot_top
set_option -output_base_name robot
set_option -enable_dsrm 1
set_option -use_cpu_as_gpio 1
# SUG1220: placement mode2 prioritizes timing without relaxing clock targets.
set_option -place_option 2
run all
