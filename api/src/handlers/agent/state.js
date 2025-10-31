import { Annotation } from '@langchain/langgraph';
import { z } from 'zod';

/* Chat Agent State and Annotations */

export const ChatAgentStateAnnotation = Annotation.Root({
  sessionId: Annotation(),
  messages: Annotation({
    reducer: (x, y) => x.concat(y),
  }),
  actions: Annotation({
    reducer: (x, y) => ({ ...x, ...y }),
  }),
  responses: Annotation({
    reducer: (x, y) => x.concat(y),
  }),
});

const ChatOrchestratorActions = z.enum(['logAnalysis', 'chatOrHelp']);

export const ChatOrchestratorAnnotation = z.object(
  Object.fromEntries(
    ChatOrchestratorActions.options.map(action => [
      action, // Key: action name
      z
        .number()
        .min(0)
        .max(1), // Value: relevancy score schema
    ]),
  ),
);

/* Log Analysis State and Annotations */

export const LogAnalysisStateAnnotation = Annotation.Root({
  sessionId: Annotation(),
  messages: Annotation({
    reducer: (x, y) => x.concat(y),
  }),
  responses: Annotation({
    reducer: (x, y) => x.concat(y),
  }),
  actions: Annotation({
    reducer: (x, y) => ({ ...x, ...y }),
  }),
  logData: Annotation(),
  logDocs: Annotation(),
});

export const LogAnalysisActions = z.enum([
  'trajectoriesAnalysis',
  'systemTimeAnalysis',
  'gpsAnalysis',
  'heartbeatAnalysis',
  'attitudeAnalysis',
  'paramValueAnalysis',
  'statusTextAnalysis',
]);

export const LogAnalysisAnnotation = z.object(
  Object.fromEntries(
    LogAnalysisActions.options.map(action => [
      action, // Key: action name
      z
        .number()
        .min(0)
        .max(1), // Value: relevancy score schema
    ]),
  ),
);
