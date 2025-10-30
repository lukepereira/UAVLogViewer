import {END, START, StateGraph} from "@langchain/langgraph";
import {getLLMClient} from "../../../clients/langgraph.js";
import {ChatAgentStateAnnotation} from "../state.js";

/* Nodes */

const chat = async (
    state,
) => {
  const {messages} = state;
  const systemMessage = {
    role: "system",
    content: "You are a helpful assistant. Answer the user's questions to the best of your ability.",
  };
  const prompt = [systemMessage, ...messages];

  const model = await getLLMClient("gpt-4.1");
  const responseMessage = await model.invoke(prompt);
  return {messages: [responseMessage]};
};

/* Edges */


const workflow = new StateGraph(ChatAgentStateAnnotation)
    .addNode("chat", chat)
    .addEdge(START, "chat")
    .addEdge("chat", END);

export const graph = workflow.compile();