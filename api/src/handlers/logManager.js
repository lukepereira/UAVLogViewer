import fs from 'fs';
import path from 'path';
import * as dfd from 'danfojs-node';

export class LogManager {
  static async uploadAndProcessLog(req, res) {
    console.debug('Upload request received');
    if (!req?.files?.length) {
      console.debug('No files uploaded');
      return res.status(400).json({ error: 'No file uploaded' });
    }

    // Files are stored inside a form using Deep Chat request FormData format:
    // https://deepchat.dev/docs/connect
    if (req.files) {
      console.log('Files:');
      console.log(req.files);
    }
    const filepath = req.files[0].path;
    await FileManager.postProcessLog(filepath);
    // Sends response back to Deep Chat using the Response format:
    // https://deepchat.dev/docs/connect/#Response
    res.status(200).json({ status: 'Files received successfully' });
  }

  static getStatsForSeries(rawSeries) {
    if (!rawSeries || rawSeries.length === 0) {
      return null;
    }
    const df = new dfd.Series(rawSeries);
    const describe = df.describe();
    const keys = describe?.$index;
    const values = describe?.$data;
    const stats = {};
    for (let i = 0; i < keys.length; i++) {
      stats[keys[i]] = values[i];
    }
    return stats;
  }

  static async postProcessLog(filepath) {
    // Read log file JSON from disk storage
    if (!filepath) {
      throw new Error(`No filename provided.`);
    }
    // Check if file exists
    const filePath = path.resolve(filepath);
    if (!fs.existsSync(filePath)) {
      throw new Error(`File not found: ${filePath}`);
    }
    const fullLogDataRaw = fs.readFileSync(filePath, 'utf-8');
    const logData = JSON.parse(fullLogDataRaw);

    // Process log data to extract statistics for various message types
    const stats = {
      SYSTEM_TIME: {
        time_boot_ms: FileManager.getStatsForSeries(logData?.messages?.SYSTEM_TIME?.time_boot_ms),
      },
      GLOBAL_POSITION_INT: {
        time_boot_ms: FileManager.getStatsForSeries(
          logData?.messages?.GLOBAL_POSITION_INT?.time_boot_ms,
        ),
        lat: FileManager.getStatsForSeries(logData?.messages?.GLOBAL_POSITION_INT?.lat),
        lon: FileManager.getStatsForSeries(logData?.messages?.GLOBAL_POSITION_INT?.lon),
        alt: FileManager.getStatsForSeries(logData?.messages?.GLOBAL_POSITION_INT?.alt),
        relative_alt: FileManager.getStatsForSeries(
          logData?.messages?.GLOBAL_POSITION_INT?.relative_alt,
        ),
        vx: FileManager.getStatsForSeries(logData?.messages?.GLOBAL_POSITION_INT?.vx),
        vy: FileManager.getStatsForSeries(logData?.messages?.GLOBAL_POSITION_INT?.vy),
        vz: FileManager.getStatsForSeries(logData?.messages?.GLOBAL_POSITION_INT?.vz),
        hdg: FileManager.getStatsForSeries(logData?.messages?.GLOBAL_POSITION_INT?.hdg),
      },
      GPS_RAW_INT: {
        fix_type: FileManager.getStatsForSeries(logData?.messages?.GPS_RAW_INT?.fix_type),
        lat: FileManager.getStatsForSeries(logData?.messages?.GPS_RAW_INT?.lat),
        lon: FileManager.getStatsForSeries(logData?.messages?.GPS_RAW_INT?.lon),
        alt: FileManager.getStatsForSeries(logData?.messages?.GPS_RAW_INT?.alt),
        eph: FileManager.getStatsForSeries(logData?.messages?.GPS_RAW_INT?.eph),
        epv: FileManager.getStatsForSeries(logData?.messages?.GPS_RAW_INT?.epv),
        vel: FileManager.getStatsForSeries(logData?.messages?.GPS_RAW_INT?.vel),
        cog: FileManager.getStatsForSeries(logData?.messages?.GPS_RAW_INT?.cog),
        satellites_visible: FileManager.getStatsForSeries(
          logData?.messages?.GPS_RAW_INT?.satellites_visible,
        ),
        alt_ellipsoid: FileManager.getStatsForSeries(logData?.messages?.GPS_RAW_INT?.alt_ellipsoid),
        h_acc: FileManager.getStatsForSeries(logData?.messages?.GPS_RAW_INT?.h_acc),
        v_acc: FileManager.getStatsForSeries(logData?.messages?.GPS_RAW_INT?.v_acc),
        vel_acc: FileManager.getStatsForSeries(logData?.messages?.GPS_RAW_INT?.vel_acc),
        hdg_acc: FileManager.getStatsForSeries(logData?.messages?.GPS_RAW_INT?.hdg_acc),
        yaw: FileManager.getStatsForSeries(logData?.messages?.GPS_RAW_INT?.yaw),
        time_boot_ms: FileManager.getStatsForSeries(logData?.messages?.GPS_RAW_INT?.time_boot_ms),
      },
      HEARTBEAT: {
        type: FileManager.getStatsForSeries(logData?.messages?.HEARTBEAT?.type),
        autopilot: FileManager.getStatsForSeries(logData?.messages?.HEARTBEAT?.autopilot),
        base_mode: FileManager.getStatsForSeries(logData?.messages?.HEARTBEAT?.base_mode),
        custom_mode: FileManager.getStatsForSeries(logData?.messages?.HEARTBEAT?.custom_mode),
        system_status: FileManager.getStatsForSeries(logData?.messages?.HEARTBEAT?.system_status),
        mavlink_version: FileManager.getStatsForSeries(
          logData?.messages?.HEARTBEAT?.mavlink_version,
        ),
        time_boot_ms: FileManager.getStatsForSeries(logData?.messages?.HEARTBEAT?.time_boot_ms),
      },
      ATTITUDE: {
        time_boot_ms: FileManager.getStatsForSeries(logData?.messages?.ATTITUDE?.time_boot_ms),
        roll: FileManager.getStatsForSeries(logData?.messages?.ATTITUDE?.roll),
        pitch: FileManager.getStatsForSeries(logData?.messages?.ATTITUDE?.pitch),
        yaw: FileManager.getStatsForSeries(logData?.messages?.ATTITUDE?.yaw),
        rollspeed: FileManager.getStatsForSeries(logData?.messages?.ATTITUDE?.rollspeed),
        pitchspeed: FileManager.getStatsForSeries(logData?.messages?.ATTITUDE?.pitchspeed),
        yawspeed: FileManager.getStatsForSeries(logData?.messages?.ATTITUDE?.yawspeed),
      },
      AHRS: {
        omegaIx: FileManager.getStatsForSeries(logData?.messages?.AHRS?.omegaIx),
        omegaIy: FileManager.getStatsForSeries(logData?.messages?.AHRS?.omegaIy),
        omegaIz: FileManager.getStatsForSeries(logData?.messages?.AHRS?.omegaIz),
        accel_weight: FileManager.getStatsForSeries(logData?.messages?.AHRS?.accel_weight),
        renorm_val: FileManager.getStatsForSeries(logData?.messages?.AHRS?.renorm_val),
        error_rp: FileManager.getStatsForSeries(logData?.messages?.AHRS?.error_rp),
        error_yaw: FileManager.getStatsForSeries(logData?.messages?.AHRS?.error_yaw),
        time_boot_ms: FileManager.getStatsForSeries(logData?.messages?.AHRS?.time_boot_ms),
      },
      AHRS2: {},
      AHRS3: {},
      PARAM_VALUE: {
        param_value: FileManager.getStatsForSeries(logData?.messages?.PARAM_VALUE?.param_value),
        param_type: FileManager.getStatsForSeries(logData?.messages?.PARAM_VALUE?.param_type),
        param_count: FileManager.getStatsForSeries(logData?.messages?.PARAM_VALUE?.param_count),
        param_index: FileManager.getStatsForSeries(logData?.messages?.PARAM_VALUE?.param_index),
        time_boot_ms: FileManager.getStatsForSeries(logData?.messages?.PARAM_VALUE?.time_boot_ms),
      },
      STATUSTEXT: {
        severity: FileManager.getStatsForSeries(logData?.messages?.STATUSTEXT?.severity),
        time_boot_ms: FileManager.getStatsForSeries(logData?.messages?.STATUSTEXT?.time_boot_ms),
      },
      trajectories: {},
    };
    
    // Save stats as JSON file 
    const statsFilename = `stats_${path.basename(filepath)}`;
    const statsFilePath = path.resolve(path.dirname(filepath), statsFilename);
    fs.writeFileSync(statsFilePath, JSON.stringify(stats, null, 2), 'utf-8');
    console.log(`Stats saved to ${statsFilePath}`);
  }
}
