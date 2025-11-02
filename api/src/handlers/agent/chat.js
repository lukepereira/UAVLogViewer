import { graph as agentChat } from './graph/chatWorkflow.js';

export class Agent {
  // Text messages are stored inside request body using the Deep Chat JSON format:
  // https://deepchat.dev/docs/connect
  static createChatBody(body, stream) {
    const chatBody = {
      messages: body.messages.map(message => {
        return { role: message.role === 'ai' ? 'assistant' : message.role, content: message.text };
      }),
      model: body.model,
    };
    if (stream) chatBody.stream = true;
    return chatBody;
  }

  static async chat(body, res, next) {
    const chatBody = Agent.createChatBody(body);
    if (!chatBody) {
      const error = new Error('Invalid chat body.');
      error.status = 400;
      return next(error);
    }

    const sessionId = body.sessionId;
    if (!sessionId) {
      const error = new Error('sessionId is required in the request body.');
      error.status = 400;
      return next(error);
    }

    const workflowInputs = {
      messages: chatBody?.messages || [],
      sessionId: sessionId,
    };

    console.debug('Starting Agent chat workflow for session:', sessionId);

    try {
      for await (const chunk of await agentChat.stream(workflowInputs, {
        streamMode: ['updates'],
      })) {
        const [mode, chunkData] = chunk;
        if (mode === 'updates') {
          const { generateFollowUp } = chunkData;
          if (generateFollowUp) {
            const { messages } = generateFollowUp;
            const message = messages[messages.length - 1];
            res.json({ text: message?.content });
          }
        }
      }
    } catch (error) {
      console.error('Error in Agent chat:', error);
      next(error);
    } finally {
      res.end();
    }
  }
}
