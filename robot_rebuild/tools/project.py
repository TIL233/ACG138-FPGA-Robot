from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
F = ROOT/'fpga'
files = sorted((F/'rtl').glob('*.v'))+sorted((F/'vendor').glob('*.v'))
p = ET.Element('Project')
ET.SubElement(p,'Template').text='FPGA'
ET.SubElement(p,'Version').text='5'
ET.SubElement(p,'Device',name='GW5AT-60B',pn='GW5AT-LV60PG484AC1/I0').text='gw5at60b-002'
fl=ET.SubElement(p,'FileList')
for f in files:
    ET.SubElement(fl,'File',path=f.relative_to(F).as_posix(),type='file.verilog',enable='1')
for name,typ in [('robot.cst','file.cst'),('robot.sdc','file.sdc')]:
    ET.SubElement(fl,'File',path=name,type=typ,enable='1')
ET.indent(p)
ET.ElementTree(p).write(F/'robot.gprj',encoding='UTF-8',xml_declaration=True)

pins={
 'clk50m':'F13','uart_tx':'C13','uart_rx':'B13','ir_rx':'E3',
 'imu_rx':'V15','ultra_l_rx':'U15','ultra_r_rx':'T15','lidar_rx':'R16',
 'enc_l_a':'T14','enc_l_b':'U16','enc_r_a':'N15','enc_r_b':'P16',
 'motor_l_pwm':'R17','motor_l_in1':'R18','motor_l_in2':'T18',
 'motor_r_pwm':'N13','motor_r_in1':'N14','motor_r_in2':'P14',
 'motor_stby':'R14','estop_n':'P15',
 'cam1_pwdn':'N17','cam1_rst_n':'P17','cam1_xclk':'W21','cam1_pclk':'W22',
 'cam1_href':'Y17','cam1_vsync':'Y16','cam1_sda':'AA20','cam1_scl':'AA21',
 'rgmii_tx_clk':'E18','rgmii_tx_ctl':'F19',
}
pins.update({f'cam1_data[{i}]':p for i,p in enumerate(['P20','V13','P19','R19','T21','U21','W16','W15'])})
pins.update({f'rgmii_txd[{i}]':p for i,p in enumerate(['B20','A20','C19','F18'])})
assert len(set(pins.values()))==len(pins)
inputs={'uart_rx','ir_rx','imu_rx','ultra_l_rx','ultra_r_rx','lidar_rx','enc_l_a','enc_l_b','enc_r_a','enc_r_b'}
lines=['// Ball locations from official camera/RGMII CST. P7 camera2 pins repurposed.']
for n,pin in pins.items():
    rg=n.startswith('rgmii')
    # P20 1-2 confirmed by user: Bank 11 and the IR level shifter use 2.5V.
    voltage = 1.8 if rg else (2.5 if n=='ir_rx' else 3.3)
    standard = 18 if rg else (25 if n=='ir_rx' else 33)
    pull='UP' if n in inputs else ('DOWN' if n=='estop_n' else 'NONE')
    lines += [f'IO_LOC "{n}" {pin};',
              f'IO_PORT "{n}" IO_TYPE=LVCMOS{standard} BANK_VCCIO={voltage} PULL_MODE={pull};']
(F/'robot.cst').write_text('\n'.join(lines)+'\n',encoding='utf-8')
print('Generated',F/'robot.gprj',len(files),'sources;',len(pins),'unique pins')
