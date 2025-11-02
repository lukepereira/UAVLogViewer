import path from 'path';
import fs from 'fs';
import * as fsPromises from 'node:fs/promises';

export const getMessagesString = (messages, maxMessages = 100) => {
  if (!messages || !Array.isArray(messages)) {
    return '';
  }
  const limitedMessages = maxMessages ? messages.slice(0, maxMessages) : messages;
  return limitedMessages
    .map(message => {
      if (message.role === 'user') {
        return `User: ${message.content}`;
      } else if (message.role === 'system' || message.role === 'assistant') {
        return `Assistant: ${message.content}`;
      }
      return '';
    })
    .join('\n');
};

export const getLogDataForSession = sessionId => {
  // Read log file data from disk storage
  const filename = fs.readdirSync('./uploads').find(name => name === `uav_log_${sessionId}.json`);

  if (!filename) {
    throw new Error(`Log file for sessionId ${sessionId} not found.`);
  }
  const filePath = path.resolve('./uploads', filename);
  const fullLogDataRaw = fs.readFileSync(filePath, 'utf-8');
  return JSON.parse(fullLogDataRaw);
};

export const getLogStatsForSession = sessionId => {
  // Read log stats file data from disk storage
  const statsFilename = fs
    .readdirSync('./uploads')
    .find(name => name === `stats_uav_log_${sessionId}.json`);

  if (!statsFilename) {
    throw new Error(`Log stats file for sessionId ${sessionId} not found.`);
  }
  const statsFilePath = path.resolve('./uploads', statsFilename);
  const fullLogStatsRaw = fs.readFileSync(statsFilePath, 'utf-8');
  return JSON.parse(fullLogStatsRaw);
};

export const getAvailableMessageGroupsAndTypes = fullLogStats => {
  const availableMessageGroupsAndTypes = {};
  for (const group of Object.keys(fullLogStats)) {
    availableMessageGroupsAndTypes[group] = {};
    const messageTypes = Object.keys(fullLogStats[group]);
    for (const messageType of messageTypes) {
      availableMessageGroupsAndTypes[group][messageType] = fullLogStats[group][messageType].map(
        item => ({
          data_key: item.data_key,
          description: item.description,
        }),
      );
    }
  }
  return availableMessageGroupsAndTypes;
};

export const saveGraphImage = async (graph, outfile = 'graph_image.png') => {
  console.log(`Saving graph image to ${outfile}`);
  const drawableGraph = await graph.getGraphAsync({ xray: false });
  const drawableGraphXray = await graph.getGraphAsync({ xray: true });

  const image1 = await drawableGraph.drawMermaidPng({
    withStyles: true,
    curveStyle: 'linear',
    backgroundColor: 'white',
  });
  const image2 = await drawableGraphXray.drawMermaidPng({
    withStyles: true,
    curveStyle: 'linear',
    backgroundColor: 'white',
  });

  const imageBuffer = new Uint8Array(await image1.arrayBuffer());
  await fsPromises.writeFile(outfile, imageBuffer);
  const imageBufferXray = new Uint8Array(await image2.arrayBuffer());
  await fsPromises.writeFile(outfile.replace('.png', '_xray.png'), imageBufferXray);
};
