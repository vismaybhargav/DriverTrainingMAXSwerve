package frc.robot.auto;

import java.lang.reflect.Field;
import java.util.HashMap;

import frc.robot.Constants.AutoConstants.AutoCommands;

public class AutoPaths {

	public static final Object[] R_AT_ALIGN_S1_R23 = new Object[] {
		"S1_R2_H",
		AutoCommands.R_ALIGN_REEF2_L_TAG_CMD,
		"R2_StationL",
		AutoCommands.DRIVE_BRAKE_CMD,
		"StationL_R3_H",
		AutoCommands.R_ALIGN_REEF3_L_TAG_CMD,
	};

	public static final Object[] R_NO_AT_S1_R23 = new Object[] {
		"S1_R2",
		AutoCommands.DRIVE_BRAKE_CMD,
		"R2_StationL",
		AutoCommands.DRIVE_BRAKE_CMD,
		"StationL_R3",
		AutoCommands.DRIVE_BRAKE_CMD,
	};

	public static final Object[] R_AT_ALIGN_S2_R13 = new Object[] {
		"S2_R1_H",
		AutoCommands.R_ALIGN_REEF1_L_TAG_CMD,
		"R1_StationL",
		AutoCommands.DRIVE_BRAKE_CMD,
		"StationL_R3_H",
		AutoCommands.R_ALIGN_REEF3_L_TAG_CMD,
	};

	public static final Object[] R_NO_AT_S2_R13 = new Object[] {
		"S2_R1",
		"R1_StationL",
		AutoCommands.DRIVE_BRAKE_CMD,
		"StationL_R3",
	};

	public static final Object[] R_AT_ALIGN_S3_R65 = new Object[] {
		"S3_R6_H",
		AutoCommands.R_ALIGN_REEF6_L_TAG_CMD,
		"R6_StationR",
		AutoCommands.DRIVE_BRAKE_CMD,
		"StationR_R5_H",
		AutoCommands.R_ALIGN_REEF5_L_TAG_CMD,
	};

	public static final Object[] R_NO_AT_S3_R65 = new Object[] {
		"S3_R6",
		"R6_StationR",
		AutoCommands.DRIVE_BRAKE_CMD,
		"StationR_R5",
	};

	public static final Object[] B_AT_ALIGN_S1_R23 = new Object[] {
		"S1_R2_H",
		AutoCommands.B_ALIGN_REEF2_L_TAG_CMD,
		"R2_StationL",
		AutoCommands.DRIVE_BRAKE_CMD,
		"StationL_R3_H",
		AutoCommands.B_ALIGN_REEF3_L_TAG_CMD,
	};

	public static final Object[] B_NO_AT_S1_R23 = new Object[] {
		"S1_R2",
		"R2_StationL",
		AutoCommands.DRIVE_BRAKE_CMD,
		"StationL_R3",
	};

	public static final Object[] B_AT_ALIGN_S2_R13 = new Object[] {
		"S2_R1_H",
		AutoCommands.B_ALIGN_REEF1_L_TAG_CMD,
		"R1_StationL",
		AutoCommands.DRIVE_BRAKE_CMD,
		"StationL_R3_H",
		AutoCommands.B_ALIGN_REEF3_L_TAG_CMD,
	};

	public static final Object[] B_NO_AT_S2_R13 = new Object[] {
		"S2_R1",
		AutoCommands.ELEVATOR_WAIT,
		"R1_StationL",
		AutoCommands.DRIVE_BRAKE_CMD,
		"StationL_R3",
	};

	public static final Object[] B_AT_ALIGN_S3_R65 = new Object[] {
		"S3_R6_H",
		AutoCommands.B_ALIGN_REEF6_L_TAG_CMD,
		"R6_StationR",
		AutoCommands.DRIVE_BRAKE_CMD,
		"StationR_R5_H",
		AutoCommands.B_ALIGN_REEF5_L_TAG_CMD,
	};

	public static final Object[] B_NO_AT_S3_R65 = new Object[] {
		"S3_R6",
		AutoCommands.DRIVE_BRAKE_CMD,
		"R6_StationR",
		AutoCommands.DRIVE_BRAKE_CMD,
		"StationR_R5",
		AutoCommands.DRIVE_BRAKE_CMD,
	};

	public static final Object[] B_AT_ALIGN_MATCH2 = new Object[] {
		"S2_R1_H",
		AutoCommands.B_ALIGN_REEF1_L_TAG_CMD,
		"R1_StationR",
		AutoCommands.DRIVE_BRAKE_CMD,
		"StationR_R5_H",
		AutoCommands.B_ALIGN_REEF5_L_TAG_CMD,
	};

	public static final Object[] B_MATCH2_NO_AT = new Object[] {
		"S2_R1",
		"R1_StationR",
		AutoCommands.DRIVE_BRAKE_CMD,
		"StationR_R5",
	};

	/**
	 * Get all autos declared in the file.
	 * @return hashmap of auto name and autos.
	 */
	public HashMap<String, Object[]> getAllAutos() {
		HashMap<String, Object[]> allObjArrays = new HashMap<String, Object[]>();
		Field[] allFields = this.getClass().getDeclaredFields();

		for (Field f: allFields) {
			if (f.getType().equals(Object[].class)) {
				try {
					f.setAccessible(true);
					Object[] array = (Object[]) f.get(this);
					allObjArrays.put(f.getName(), array);
				} catch (IllegalAccessException e) {
					e.printStackTrace();
				}
			}
		}

		return allObjArrays;
	}
}
