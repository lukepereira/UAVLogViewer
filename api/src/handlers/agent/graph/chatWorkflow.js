import { END, START, StateGraph, Send } from '@langchain/langgraph';
import { getLLMClient } from '../../../clients/langgraph.js';
import { ChatAgentStateAnnotation, ChatOrchestratorAnnotation } from '../state.js';
import { getMessagesString } from '../utils.js';
import { graph as logAnalysisGraph } from './logAnalysis.js';
import {
  getChatOrchestratorPrompt,
  getWorkflowSynthesisPrompt,
  getFollowUpPrompt,
  getChatHelpPrompt,
} from '../prompts/chatWorkflow.js';
import { noDataFoundMessage } from '../prompts/common.js';
import { getAllLogDocumentation } from '../prompts/logDocumentation.js';

/* Nodes */

const orchestrator = async state => {
  console.debug('Orchestrator node invoked');
  const { messages } = state;

  // Prepare prompt
  const messagesString = getMessagesString(messages);
  const orchestratorPrompt = getChatOrchestratorPrompt(messagesString);
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
    .withStructuredOutput(ChatOrchestratorAnnotation)
    .invoke(orchestratorMessages);
  console.log('Orchestrator actions:', responseMessage);
  return { actions: responseMessage };
};

const chatOrHelp = async state => {
  console.debug('chatOrHelp node invoked');
  const { messages } = state;

  // Prepare prompt
  const formattedMessages = getMessagesString(messages);
  const documentationString = getAllLogDocumentation();
  const helpPrompt = getChatHelpPrompt(formattedMessages, documentationString);
  const helpMessages = [
    {
      role: 'system',
      content: helpPrompt,
    },
  ];

  // Get help response from LLM
  const model = await getLLMClient('gpt-4.1-mini');
  const responseMessage = await model.invoke(helpMessages);
  return { responses: [{ ...responseMessage, node: 'chatOrHelp' }] };
};

const synthesizeResponse = async state => {
  console.debug('synthesizeResponse node invoked');

  const { messages, responses } = state;
  if (!responses || responses.length === 0) {
    console.debug('No responses to synthesize, returning no data found message.');
    return {
      responses: [{ role: 'assistant', node: 'synthesizeResponse', content: noDataFoundMessage }],
    };
  }

  if (responses.length === 1) {
    console.debug('Only one response, skipping synthesis.');
    return { responses: [{ ...responses[0], node: 'synthesizeResponse' }] };
  }

  const formattedMessages = getMessagesString(messages);
  const formattedResponses = responses.map(response => response.content).join('\n');
  const synthesisPrompt = getWorkflowSynthesisPrompt(formattedMessages, formattedResponses);
  const synthesisMessages = [
    {
      role: 'system',
      content: synthesisPrompt,
    },
  ];
  const model = await getLLMClient('gpt-4.1-mini');
  const responseMessage = await model.invoke(synthesisMessages);
  return { responses: [{ ...responseMessage, node: 'synthesizeResponse' }] };
};

const generateFollowUp = async state => {
  console.debug('generateFollowUp node invoked');
  const { messages, responses } = state;
  const formattedMessages = getMessagesString(messages);
  const synthesizedResponse = responses
    .filter(response => response.node === 'synthesizeResponse')
    .map(response => response.content)
    .join(' ');
  const helpResponse = responses
    .filter(response => response.node === 'chatOrHelp')
    .map(response => response.content)
    .join(' ');

  // Generate follow up using both synthesizedResponse and helpResponse
  const followUpPrompt = getFollowUpPrompt(formattedMessages, synthesizedResponse, helpResponse);

  const followUpMessages = [
    {
      role: 'system',
      content: followUpPrompt,
    },
  ];
  const model = await getLLMClient('gpt-4.1-mini');
  const responseMessage = await model.invoke(followUpMessages);

  // Concatenate helpResponse, synthesizedResponse and follow-up into single message
  let content = '';
  if (helpResponse && helpResponse.trim().length > 0) {
    content += helpResponse + '\n\n';
  }
  if (synthesizedResponse && synthesizedResponse.trim().length > 0) {
    content += synthesizedResponse + '\n\n';
  }
  content += responseMessage.content;
  return { messages: [{ role: 'assistant', node: 'generateFollowUp', content }] };
};

/* Edges */

const routeOrchestratorOutput = async state => {
  console.debug('Routing orchestrator output');
  const { messages, actions, sessionId } = state;
  const threshold = 0.5;

  // If no actions exceed threshold, default to chatOrHelp
  let anyActionAboveThreshold = false;
  for (const score of Object.values(actions)) {
    if (score >= threshold) {
      anyActionAboveThreshold = true;
      break;
    }
  }
  if (!anyActionAboveThreshold) {
    console.debug('No actions above threshold, defaulting to chatOrHelp');
    return new Send('chatOrHelp', { messages });
  }

  // Route all actions that are scored above threshold
  return Object.entries(actions).reduce((sends, [action, score]) => {
    if (score >= threshold) {
      if (action === 'logAnalysis') {
        sends.push(new Send('logAnalysis', { messages, sessionId }));
      }
      if (action === 'chatOrHelp') {
        console.debug('Routing to chatOrHelp based on orchestrator action');
        sends.push(new Send('chatOrHelp', { messages }));
      }
    }
    return sends;
  }, []);
};

const workflow = new StateGraph(ChatAgentStateAnnotation)
  .addNode('orchestrator', orchestrator)
  .addNode('logAnalysis', logAnalysisGraph)
  .addNode('chatOrHelp', chatOrHelp)
  .addNode('synthesizeResponse', synthesizeResponse)
  .addNode('generateFollowUp', generateFollowUp)
  .addEdge(START, 'orchestrator')
  .addConditionalEdges('orchestrator', routeOrchestratorOutput)
  .addEdge('logAnalysis', 'synthesizeResponse')
  .addEdge('chatOrHelp', 'generateFollowUp')
  .addEdge('synthesizeResponse', 'generateFollowUp')
  .addEdge('generateFollowUp', END);

export const graph = workflow.compile();

// Uncomment to save graph image
// import { saveGraphImage } from "../utils.js";
// saveGraphImage(graph, './agent_workflow.png');
