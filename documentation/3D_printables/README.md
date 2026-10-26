# 3D Printable Parts

This directory contains the FreeCAD source models and STEP exports. Use the STEP files for slicers and other CAD software; use the FreeCAD files when you want to inspect or edit the models.

## Parts list

| Quantity | STEP file | FreeCAD file | Remarks |
| ---: | --- | --- | --- |
| 1 | [Back_shell_txt_v2_1-Body.step](STEP/Back_shell_txt_v2_1-Body.step) | [FreeCAD](FreeCAD/Back_shell_txt_v2_1.FCStd) | Definitely do not forget to enable support when slicing it. |
| 1 | [buzzer_cover-Body.step](STEP/buzzer_cover-Body.step) | [FreeCAD](FreeCAD/buzzer_cover.FCStd) | — |
| 1 | [connector_cover_v2-Body.step](STEP/connector_cover_v2-Body.step) | [FreeCAD](FreeCAD/connector_cover_v2.FCStd) | This covers the top connectors (USB-C & HDMI). It can be just pushed inside and should hold. |
| 2 | [DistanceBar-Body.step](STEP/DistanceBar-Body.step) | [FreeCAD](FreeCAD/DistanceBar.FCStd) | — |
| 1 | [Front_shell_v2-Body.step](STEP/Front_shell_v2-Body.step) | [FreeCAD](FreeCAD/Front_shell_v2.FCStd) | — |
| 1 | [power_cord_holder-Body.step](STEP/power_cord_holder-Body.step) | [FreeCAD](FreeCAD/power_cord_holder.FCStd) | This allows to have the power cord through the back side. After inserting the power cord, close the opening with this part. |
| 4 | [ScrewStopper-Body.step](STEP/ScrewStopper-Body.step) | [FreeCAD](FreeCAD/ScrewStopper.FCStd) | — |
| 1 | [side_hatch_v2_1-Body.step](STEP/side_hatch_v2_1-Body.step) | [FreeCAD](FreeCAD/side_hatch_v2_1.FCStd) | This should be fitted to the side of  "Front_shell_v2-Body.step". It may be difficult to insert without modification after printing |
| 1 | [side_hatch_v2_lock-Body.step](STEP/side_hatch_v2_lock-Body.step) | [FreeCAD](FreeCAD/side_hatch_v2_lock.FCStd) | — |
| 1 | [SSD_holder-Body.step](STEP/SSD_holder-Body.step) | [FreeCAD](FreeCAD/SSD_holder.FCStd) | Only if you use this SSD adapter: WAVESHARE-27580 (PCIe To M.2 Mini Adapter) and this SSD: Raspberry Pi official SSD 256 GB |
| 1 | [switch_cap-Body.step](STEP/switch_cap-Body.step) | [FreeCAD](FreeCAD/switch_cap.FCStd) | Only if you use an CherryMX keyboard swith type to switch on and off the Raspberry Pi. |
| 1 | [SwitchCover_V3-Body.step](STEP/SwitchCover_V3-Body.step) | [FreeCAD](FreeCAD/SwitchCover_V3.FCStd) | Only if you use an CherryMX keyboard swith type to switch on and off the Raspberry Pi. |
| 1 | [TopHatchV2-Body.step](STEP/TopHatchV2-Body.step) | [FreeCAD](FreeCAD/TopHatchV2.FCStd) | This should easy fit into the top connector of the Front_shell |
| 1 | [tilt_stand_V1-Body.step](STEP/tilt_stand_V1-Body.step) | [FreeCAD](FreeCAD/tilt_stand_V1.FCStd) | Keeps the device tilted for improved usability |

## Recommended slicer settings

The provided parts list recommends these settings:

- Wall loops: 3
- Sparse infill density: 15%
- Infill pattern: Rectilinear
- Layer height: 0.2 mm
- Supports: enabled
- Support type: Tree (auto)

The parts list specifically emphasizes enabling supports for `Back_shell_txt_v2_1-Body.step`.

## Models not listed in the supplied parts list

These model files are present in the repository but are not included in the parts list:

- [IC_case_base_V3-Body.step](STEP/IC_case_base_V3-Body.step) · [FreeCAD](FreeCAD/IC_case_base_V3.FCStd)
- [IC_case_top_V3-Body.step](STEP/IC_case_top_V3-Body.step) · [FreeCAD](FreeCAD/IC_case_top_V3.FCStd)
