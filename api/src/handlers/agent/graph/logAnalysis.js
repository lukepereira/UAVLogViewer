import { END, START, StateGraph, Send } from '@langchain/langgraph';
import { getLLMClient } from '../../../clients/langgraph.js';
import { LogAnalysisStateAnnotation, LogAnalysisAnnotation } from '../state.js';
import { getMessagesString, getLogDataForSession, getLogStatsForSession } from '../utils.js';
import { getAnalysisOrchestratorPrompt, getLogAnalysisPrompt } from '../prompts/logAnalysis.js';
import {
  getSystemTimeDocumentation,
  getGpsDocumentation,
  getHeartbeatDocumentation,
  getAttitudeDocumentation,
  getParamValueDocumentation,
  getStatusTextDocumentation,
  getTrajectoriesDocumentation,
} from '../prompts/logDocumentation.js';

/* Nodes */

const logAnalysisOrchestrator = async state => {
  console.debug('logAnalysisOrchestrator node invoked');
  const { messages } = state;

  // Prepare prompt for orchestrator
  const messagesString = getMessagesString(messages);
  const orchestratorPrompt = getAnalysisOrchestratorPrompt(messagesString);
  const orchestratorMessages = [
    {
      role: 'system',
      content: orchestratorPrompt,
    },
    {
      role: 'user',
      content: messagesString,
    },
  ];

  // Get structured orchestrator scores from LLM
  const model = await getLLMClient('gpt-4.1-mini');
  const responseMessage = await model
    .withStructuredOutput(LogAnalysisAnnotation)
    .invoke(orchestratorMessages);
  return { actions: responseMessage };
};

const genericAnalysis = async state => {
  const { messages, logData, logDocs, logStats } = state;
  const maxLogLimit = 128000;
  const formattedMessages = getMessagesString(messages);
  const logStatsString = JSON.stringify(logStats);

  // Truncate log data if too large
  let logDataString = JSON.stringify(logData);
  if (logDataString.length > maxLogLimit) {
    logDataString = logDataString.slice(0, maxLogLimit);
    logDataString += '\n... [truncated due to size]';
  }
  const logAnalysisPrompt = getLogAnalysisPrompt(
    formattedMessages,
    logDataString,
    logStatsString,
    logDocs,
  );
  const logAnalysisMessages = [
    {
      role: 'system',
      content: logAnalysisPrompt,
    },
  ];
  const model = await getLLMClient('gpt-4.1-mini');
  const responseMessage = await model.invoke(logAnalysisMessages);
  return responseMessage;
};

const trajectoriesAnalysis = async state => {
  console.debug('trajectoriesAnalysis node invoked');
  const responseMessage = await genericAnalysis(state);
  return { responses: [{ ...responseMessage, node: 'trajectoriesAnalysis' }] };
};

const systemTimeAnalysis = async state => {
  console.debug('systemTimeAnalysis node invoked');
  const responseMessage = await genericAnalysis(state);
  return { responses: [{ ...responseMessage, node: 'systemTimeAnalysis' }] };
};

const gpsAnalysis = async state => {
  console.debug('gpsAnalysis node invoked');
  const responseMessage = await genericAnalysis(state);
  return { responses: [{ ...responseMessage, node: 'gpsAnalysis' }] };
};

const heartbeatAnalysis = async state => {
  console.debug('heartbeatAnalysis node invoked');
  const responseMessage = await genericAnalysis(state);
  return { responses: [{ ...responseMessage, node: 'heartbeatAnalysis' }] };
};

const attitudeAnalysis = async state => {
  console.debug('attitudeAnalysis node invoked');
  const responseMessage = await genericAnalysis(state);
  return { responses: [{ ...responseMessage, node: 'attitudeAnalysis' }] };
};

const paramValueAnalysis = async state => {
  console.debug('paramValueAnalysis node invoked');
  const responseMessage = await genericAnalysis(state);
  return { responses: [{ ...responseMessage, node: 'paramValueAnalysis' }] };
};

const statusTextAnalysis = async state => {
  console.debug('statusTextAnalysis node invoked');
  const responseMessage = await genericAnalysis(state);
  return { responses: [{ ...responseMessage, node: 'statusTextAnalysis' }] };
};

/* Edges */

const routeOrchestratorOutput = async state => {
  console.debug('Routing analysis orchestrator output');
  const { messages, actions, sessionId } = state;
  console.debug('Orchestrator actions:', actions);
  const threshold = 0.5;

  // If no actions exceed threshold, do not route any analyses
  let anyActionAboveThreshold = false;
  for (const score of Object.values(actions)) {
    if (score >= threshold) {
      anyActionAboveThreshold = true;
      break;
    }
  }
  if (!anyActionAboveThreshold) {
    return {};
  }

  // Read log file data from disk storage
  let fullLogData;
  try {
    fullLogData = getLogDataForSession(sessionId);
  } catch (error) {
    console.error('Error reading log file:', error);
    return {};
  }

  // Read log stats file data from disk storage
  let fullLogStats;
  try {
    fullLogStats = getLogStatsForSession(sessionId);
  } catch (error) {
    console.error('Error reading log stats file:', error);
    return {};
  }

  // Route all actions above threshold, prepare relevant log data and documentation
  // TODO: DRY this up
  return Object.entries(actions).reduce((sends, [action, score]) => {
    if (score >= threshold) {
      if (action === 'trajectoriesAnalysis') {
        const logData = {
          ...fullLogData?.trajectories,
        };
        const logStats = fullLogStats?.trajectories;
        const logDocs = getTrajectoriesDocumentation();
        sends.push(new Send('trajectoriesAnalysis', { messages, logData, logDocs, logStats }));
      }
      if (action === 'systemTimeAnalysis') {
        const logData = {
          ...fullLogData?.messages?.SYSTEM_TIME,
        };
        const logStats = fullLogStats?.SYSTEM_TIME;
        const logDocs = getSystemTimeDocumentation();
        sends.push(new Send('systemTimeAnalysis', { messages, logData, logDocs, logStats }));
      }
      if (action === 'gpsAnalysis') {
        const logData = {
          ...fullLogData?.messages?.GLOBAL_POSITION_INT,
          ...fullLogData?.messages?.GPS_RAW_INT,
        };
        const logStats = {
          ...fullLogStats?.GPS_RAW_INT,
          ...fullLogStats?.GLOBAL_POSITION_INT,
        };
        const logDocs = getGpsDocumentation();
        sends.push(new Send('gpsAnalysis', { messages, logData, logDocs, logStats }));
      }
      if (action === 'heartbeatAnalysis') {
        const logData = {
          ...fullLogData?.messages?.HEARTBEAT,
        };
        const logStats = fullLogStats?.HEARTBEAT;
        const logDocs = getHeartbeatDocumentation();
        sends.push(new Send('heartbeatAnalysis', { messages, logData, logDocs, logStats }));
      }
      if (action === 'attitudeAnalysis') {
        const logData = {
          ...fullLogData?.messages?.ATTITUDE,
          ...fullLogData?.messages?.AHRS,
          ...fullLogData?.messages?.AHRS2,
          ...fullLogData?.messages?.AHRS3,
        };
        const logStats = {
          ...fullLogStats?.ATTITUDE,
          ...fullLogStats?.AHRS,
          ...fullLogStats?.AHRS2,
          ...fullLogStats?.AHRS3,
        };
        const logDocs = getAttitudeDocumentation();
        sends.push(new Send('attitudeAnalysis', { messages, logData, logDocs, logStats }));
      }
      if (action === 'paramValueAnalysis') {
        const logData = {
          ...fullLogData?.messages?.PARAM_VALUE,
        };
        const logStats = fullLogStats?.PARAM_VALUE;
        const logDocs = getParamValueDocumentation();
        sends.push(new Send('paramValueAnalysis', { messages, logData, logDocs, logStats }));
      }
      if (action === 'statusTextAnalysis') {
        const logData = {
          ...fullLogData?.messages?.STATUSTEXT,
        };
        const logStats = fullLogStats?.STATUSTEXT;
        const logDocs = getStatusTextDocumentation();
        sends.push(new Send('statusTextAnalysis', { messages, logData, logDocs, logStats }));
      }
    }
    return sends;
  }, []);
};

const workflow = new StateGraph(LogAnalysisStateAnnotation)
  .addNode('logAnalysisOrchestrator', logAnalysisOrchestrator)
  .addNode('trajectoriesAnalysis', trajectoriesAnalysis)
  .addNode('systemTimeAnalysis', systemTimeAnalysis)
  .addNode('gpsAnalysis', gpsAnalysis)
  .addNode('heartbeatAnalysis', heartbeatAnalysis)
  .addNode('attitudeAnalysis', attitudeAnalysis)
  .addNode('paramValueAnalysis', paramValueAnalysis)
  .addNode('statusTextAnalysis', statusTextAnalysis)
  .addEdge(START, 'logAnalysisOrchestrator')
  .addConditionalEdges('logAnalysisOrchestrator', routeOrchestratorOutput)
  .addEdge('trajectoriesAnalysis', END)
  .addEdge('systemTimeAnalysis', END)
  .addEdge('gpsAnalysis', END)
  .addEdge('heartbeatAnalysis', END)
  .addEdge('attitudeAnalysis', END)
  .addEdge('paramValueAnalysis', END)
  .addEdge('statusTextAnalysis', END);

export const graph = workflow.compile();
