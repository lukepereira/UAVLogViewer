import { END, START, StateGraph, Send } from '@langchain/langgraph';
import { getLLMClient } from '../../../clients/langgraph.js';
import { LogAnalysisStateAnnotation, LogAnalysisAnnotation } from '../state.js';
import { getMessagesString, getLogDataForSession, getLogStatsForSession } from '../utils.js';
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

  // Include info about available message groups and types in log data
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
  const availableMessageGroupsAndTypesString = JSON.stringify(availableMessageGroupsAndTypes);

  // Prepare prompt for orchestrator
  const messagesString = getMessagesString(messages);
  const orchestratorPrompt = getAnalysisOrchestratorPrompt(
    messagesString,
    availableMessageGroupsAndTypesString,
  );
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
  const { messages, logContext } = state;
  const maxLogLimit = 128000;
  const formattedMessages = getMessagesString(messages);

  // Truncate log data if too large
  let logContextString = JSON.stringify(logContext);
  if (logContextString.length > maxLogLimit) {
    logContextString = logContextString.slice(0, maxLogLimit);
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
  const responseMessage = await genericAnalysis(state);
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

  // Read log stats file data from disk storage
  let fullLogStats;
  try {
    fullLogStats = getLogStatsForSession(sessionId);
  } catch (error) {
    console.error('Error reading log stats file:', error);
    return {};
  }

  // Route all actions above threshold, prepare relevant log data and documentation
  return Object.entries(actions).reduce((sends, [action, score]) => {
    if (score >= threshold) {
      const logContext = fullLogStats?.[action] || {};
      sends.push(new Send(action, { messages, logContext }));
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
