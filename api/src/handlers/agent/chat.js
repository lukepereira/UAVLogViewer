import {graph as agentChat} from "./graph/agentWorkflow.js";

export class Agent {
    static createChatBody(body, stream) {
    // Text messages are stored inside request body using the Deep Chat JSON format:
    // https://deepchat.dev/docs/connect
    const chatBody = {
      messages: body.messages.map((message) => {
        return {role: message.role === 'ai' ? 'assistant' : message.role, content: message.text};
      }),
      model: body.model,
    };
    if (stream) chatBody.stream = true;
    return chatBody;
  }

  static async chat(body, res, next) {
    const chatBody = Agent.createChatBody(body);
    const workflowInputs = {
        messages: chatBody?.messages || [],
    }

    try {
        for await (
            const chunk of await agentChat.stream(workflowInputs, {
                streamMode: ["values"],
            })
        ) {
            const messages = chunk?.[1]?.messages;
            if (messages && messages.length > 1) {
                const message = messages[messages.length - 1];
                // Sends response back to Deep Chat using the Response format:
                // https://deepchat.dev/docs/connect/#Response
                res.json({text: message?.content});
            }
        }
        } catch (error) {
            console.error("Error in Agent chat:", error);
            next(error);
        } finally {
            res.end();
        }
    }
}