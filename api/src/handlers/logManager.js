import fs from 'fs';
import path from 'path';
import * as ss from 'simple-statistics';
import {
  logDataMessageGroups,
  messageDataDocs_bin,
  messageDataDocs_tlog,
} from './agent/prompts/logDocumentation.js';

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
      console.debug('Files:', req.files);
    }
    // Sends response back to Deep Chat using the Response format:
    // https://deepchat.dev/docs/connect/#Response
    res.status(200).json({ status: 'Files received successfully' });

    // Process the uploaded log file in the background
    const filepath = req.files[0].path;
    await LogManager.preProcessLog(filepath);
  }

  static getStatsForMessageField(messageFieldData, timeBootMsData = undefined) {
    if (!messageFieldData || messageFieldData.length === 0) {
      return null;
    }
    let rawSeries = messageFieldData;

    if (typeof messageFieldData === 'object' && !Array.isArray(messageFieldData)) {
      rawSeries = Object.values(messageFieldData);
    }

    if (!Array.isArray(rawSeries)) {
      console.debug('Input is not an array');
      return null;
    }

    const stats = {
      count: rawSeries.length,
      mean: ss.mean(rawSeries),
      std: ss.standardDeviation(rawSeries),
      min: ss.min(rawSeries),
      max: ss.max(rawSeries),
      median: ss.median(rawSeries),
      q25: ss.quantile(rawSeries, 0.25),
      q75: ss.quantile(rawSeries, 0.75),
    };

    if (!timeBootMsData) {
      return stats;
    }
    // If time series data is provided, compute trend line and R²
    let timeBootMsArray = timeBootMsData;
    if (typeof timeBootMsData === 'object' && !Array.isArray(timeBootMsData)) {
      timeBootMsArray = Object.values(timeBootMsData);
    }
    if (!Array.isArray(timeBootMsArray)) {
      console.debug('timeBootMsData is not an array');
      return stats;
    }
    if (timeBootMsArray.length !== rawSeries.length) {
      console.debug('timeBootMsData length does not match messageFieldData length');
      return stats;
    }
    const xy = timeBootMsArray.map((t, i) => [t, rawSeries[i]]);
    const regression = ss.linearRegression(xy);
    const line = ss.linearRegressionLine(regression);
    const r2 = ss.rSquared(xy, line);

    stats.trend = {
      slope: regression.m,
      intercept: regression.b,
      r2: r2,
    };

    return stats;
  }

  static getDescriptionForMessageField(logType, messageType, fieldKey) {
    const messageDataDocs = logType === 'bin' ? messageDataDocs_bin : messageDataDocs_tlog;
    let desc = `Placeholder description for ${messageType}.${fieldKey}`;
    const messageDoc = messageDataDocs.find(
      doc => doc.title === messageType || doc.id === messageType.toLowerCase(),
    );

    if (messageDoc) {
      let fieldDoc = null;
      for (const table of messageDoc.tables) {
        const foundField = table.find(field => field.name === fieldKey);
        if (foundField) {
          fieldDoc = foundField;
          break;
        }
      }

      if (fieldDoc) {
        desc = `${fieldDoc?.description} (Unit: ${fieldDoc?.unit})`;
      } else {
        desc = messageDoc.description || '';
      }
    }
    return desc;
  }

  static getRawDataForMessageField(messageFieldData, timeBootMsData = undefined) {
    if (!messageFieldData || messageFieldData.length === 0) {
      return null;
    }
    let rawData = messageFieldData;
    if (typeof messageFieldData === 'object' && !Array.isArray(messageFieldData)) {
      rawData = Object.values(messageFieldData);
    }

    // If data repeats itself, only return unique value to reduce size
    if (Array.isArray(rawData)) {
      const uniqueData = Array.from(new Set(rawData));
      if (uniqueData.length < rawData.length) {
        return `${uniqueData} ... [truncated, ${rawData.length -
          uniqueData.length} more repeated values]`;
      }
    }

    // if timeBootMsData is provided, merge it with messageFieldData for time context
    if (timeBootMsData) {
      let timeBootMsArray = timeBootMsData;
      if (typeof timeBootMsData === 'object' && !Array.isArray(timeBootMsData)) {
        timeBootMsArray = Object.values(timeBootMsData);
      }
      if (!Array.isArray(timeBootMsArray)) {
        console.debug('timeBootMsData is not an array');
        return rawData;
      }
      if (timeBootMsArray.length !== rawData.length) {
        console.debug('timeBootMsData length does not match messageFieldData length');
        return rawData;
      }

      const combinedData = rawData.map((value, index) => {
        return [`${timeBootMsArray[index]}ms`, value];
      });
      return combinedData;
    }

    return rawData;
  }

  static async preProcessLog(filepath) {
    // Placeholder for any preprocessing steps if needed in the future
    console.debug(`Preprocessing log file at ${filepath}`);
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
    const logType = logData?.logType;
    if (!['tlog', 'bin'].includes(logType)) {
      throw new Error(`Unsupported log type: ${logType}`);
    }

    const logStats = {};

    for (const [groupName, groupInfo] of Object.entries(logDataMessageGroups)) {
      const validMessages = logType === 'tlog' ? groupInfo.tlog : groupInfo.bin;
      if (!validMessages) continue;

      logStats[groupName] = {};

      for (const messageType of validMessages) {
        const messageData = logData?.messages?.[messageType];
        if (!messageData) continue;

        const dataKeys = Object.keys(messageData);
        logStats[groupName][messageType] = dataKeys.map(key => ({
          data_key: key,
          description: LogManager.getDescriptionForMessageField(logType, messageType, key),
          stats: LogManager.getStatsForMessageField(
            messageData[key],
            messageData?.['time_boot_ms'],
          ),
          rawData: LogManager.getRawDataForMessageField(
            messageData[key],
            messageData?.['time_boot_ms'],
          ),
        }));
      }
    }

    // Save stats and doc contexts as JSON files
    const statsFilename = `stats_${path.basename(filepath)}`;
    const statsFilePath = path.resolve(path.dirname(filepath), statsFilename);
    fs.writeFileSync(statsFilePath, JSON.stringify(logStats, null, 2), 'utf-8');
    console.log(`Stats saved to ${statsFilePath}`);
    return;
  }
}
