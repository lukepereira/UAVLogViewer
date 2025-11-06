import { END, START, StateGraph, Send } from '@langchain/langgraph';
import { getLLMClient } from '../../../clients/langgraph.js';
import { LogAnalysisStateAnnotation, LogAnalysisAnnotation } from '../state.js';
import {
  getMessagesString,
  getLogStatsForSession,
  getLogDataForSession,
} from '../utils/analysisUtils.js';
import { getAnalysisOrchestratorPrompt, getLogAnalysisPrompt } from '../prompts/logAnalysis.js';

/* Nodes */

const logAnalysisOrchestrator = async state => {
  console.debug('logAnalysisOrchestrator node invoked');
  const { sessionId, messages } = state;

  // Read log stats file data from disk storage
  let fullLogStats;
  try {
    fullLogStats = getLogStatsForSession(sessionId);
  } catch (error) {
    console.error('Error reading log stats file:', error);
    return {};
  }

  // Prepare prompt for orchestrator
  const messagesString = getMessagesString(messages);
  const orchestratorPrompt = getAnalysisOrchestratorPrompt(messagesString);
  const orchestratorMessages = [
    {
      role: 'system',
      content: orchestratorPrompt,
    },
  ];

  // Get structured orchestrator scores from LLM
  const model = await getLLMClient('gpt-4.1-mini');
  const responseMessage = await model
    .withStructuredOutput(LogAnalysisAnnotation)
    .invoke(orchestratorMessages);
  return { actions: responseMessage };
};

const genericAnalysis = async (state, additionalContext = undefined) => {
  const { messages, logContext } = state;
  const formattedMessages = getMessagesString(messages);

  // If additionalContext is provided, merge it into logContext
  if (additionalContext) {
    for (const messageType of Object.keys(additionalContext)) {
      if (!logContext[messageType]) {
        logContext[messageType] = additionalContext[messageType];
      } else {
        logContext[messageType] = logContext[messageType].concat(additionalContext[messageType]);
      }
    }
  }

  // Truncate log context string if too large
  let logContextString = JSON.stringify(logContext);
  const maxContextLength = 256000;
  if (logContextString.length > maxContextLength) {
    console.debug(
      `Truncating log context for analysis due to size: ${logContextString.length} > ${maxContextLength}`,
    );
    logContextString = logContextString.slice(0, maxContextLength);
    logContextString += '\n... [truncated due to size]';
  }

  const logAnalysisPrompt = getLogAnalysisPrompt(formattedMessages, logContextString);
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

const inertialMotion = async state => {
  console.debug('inertialMotion node invoked');
  const responseMessage = await genericAnalysis(state);
  return { responses: [{ ...responseMessage, node: 'inertialMotion' }] };
};

const gpsNavigation = async state => {
  console.debug('gpsNavigation node invoked');

  // Read trajectories data and include as additional context for gpsNavigation analysis
  let additionalContext = {};
  try {
    const { sessionId } = state;
    const logData = getLogDataForSession(sessionId);
    const trajectoriesData = logData?.trajectories;
    if (trajectoriesData) {
      additionalContext['trajectories'] = [
        {
          data_key: 'trajectories',
          description: 'Summarized trajectory data',
          rawData: trajectoriesData,
        },
      ];
    } else {
      console.debug('No trajectories data found for additional context');
    }
  } catch (error) {
    console.error('Error reading log data file for trajectories:', error);
  }

  const responseMessage = await genericAnalysis(state, additionalContext);
  return { responses: [{ ...responseMessage, node: 'gpsNavigation' }] };
};

const controlAutopilot = async state => {
  console.debug('controlAutopilot node invoked');
  const responseMessage = await genericAnalysis(state);
  return { responses: [{ ...responseMessage, node: 'controlAutopilot' }] };
};

const actuatorsMotors = async state => {
  console.debug('actuatorsMotors node invoked');
  const responseMessage = await genericAnalysis(state);
  return { responses: [{ ...responseMessage, node: 'actuatorsMotors' }] };
};

const airEnvironment = async state => {
  console.debug('airEnvironment node invoked');
  const responseMessage = await genericAnalysis(state);
  return { responses: [{ ...responseMessage, node: 'airEnvironment' }] };
};

const simulationSitl = async state => {
  console.debug('simulationSitl node invoked');
  const responseMessage = await genericAnalysis(state);
  return { responses: [{ ...responseMessage, node: 'simulationSitl' }] };
};

const communicationTelemetry = async state => {
  console.debug('communicationTelemetry node invoked');
  const responseMessage = await genericAnalysis(state);
  return { responses: [{ ...responseMessage, node: 'communicationTelemetry' }] };
};

const loggingReplay = async state => {
  console.debug('loggingReplay node invoked');
  const responseMessage = await genericAnalysis(state);
  return { responses: [{ ...responseMessage, node: 'loggingReplay' }] };
};

const powerBattery = async state => {
  console.debug('powerBattery node invoked');
  const responseMessage = await genericAnalysis(state);
  return { responses: [{ ...responseMessage, node: 'powerBattery' }] };
};

const camerasSensors = async state => {
  console.debug('camerasSensors node invoked');
  const responseMessage = await genericAnalysis(state);
  return { responses: [{ ...responseMessage, node: 'camerasSensors' }] };
};

const safetyMisc = async state => {
  console.debug('safetyMisc node invoked');
  const responseMessage = await genericAnalysis(state);
  return { responses: [{ ...responseMessage, node: 'safetyMisc' }] };
};

/* Edges */

const routeOrchestratorOutput = async state => {
  console.debug('Routing analysis orchestrator output');
  const { messages, actions, sessionId } = state;
  const threshold = 0.5;
  console.debug('Orchestrator action scores:', actions);

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

  // Read log stats file data from disk storage
  let fullLogStats;
  try {
    fullLogStats = getLogStatsForSession(sessionId);
  } catch (error) {
    console.error('Error reading log stats file:', error);
    return {};
  }

  const availableMessageGroups = [];
  for (const group of Object.keys(fullLogStats)) {
    // GPS data is always available
    if (group === 'gpsNavigation') {
      availableMessageGroups.push(group);
      continue;
    }
    // check if group has any non-empty message types
    const messageTypes = Object.keys(fullLogStats[group]);
    let hasNonEmptyTypes = false;
    for (const messageType of messageTypes) {
      if (fullLogStats[group][messageType].length > 0) {
        hasNonEmptyTypes = true;
        break;
      }
    }
    if (hasNonEmptyTypes) {
      availableMessageGroups.push(group);
    }
  }
  console.debug('Available message groups in log data:', availableMessageGroups);

  // Route all actions above threshold and available in log data
  // include preprocessed log stats and docs as context
  return Object.entries(actions).reduce((sends, [action, score]) => {
    if (score >= threshold && availableMessageGroups.includes(action)) {
      const logContext = fullLogStats?.[action] || {};
      sends.push(new Send(action, { messages, logContext, sessionId }));
    }
    return sends;
  }, []);
};

const workflow = new StateGraph(LogAnalysisStateAnnotation)
  .addNode('logAnalysisOrchestrator', logAnalysisOrchestrator)
  .addNode('inertialMotion', inertialMotion)
  .addNode('gpsNavigation', gpsNavigation)
  .addNode('controlAutopilot', controlAutopilot)
  .addNode('actuatorsMotors', actuatorsMotors)
  .addNode('airEnvironment', airEnvironment)
  .addNode('simulationSitl', simulationSitl)
  .addNode('communicationTelemetry', communicationTelemetry)
  .addNode('loggingReplay', loggingReplay)
  .addNode('powerBattery', powerBattery)
  .addNode('camerasSensors', camerasSensors)
  .addNode('safetyMisc', safetyMisc)
  .addEdge(START, 'logAnalysisOrchestrator')
  .addConditionalEdges('logAnalysisOrchestrator', routeOrchestratorOutput)
  .addEdge('inertialMotion', END)
  .addEdge('gpsNavigation', END)
  .addEdge('controlAutopilot', END)
  .addEdge('actuatorsMotors', END)
  .addEdge('airEnvironment', END)
  .addEdge('simulationSitl', END)
  .addEdge('communicationTelemetry', END)
  .addEdge('loggingReplay', END)
  .addEdge('powerBattery', END)
  .addEdge('camerasSensors', END)
  .addEdge('safetyMisc', END);

export const graph = workflow.compile();
