import {ChatOpenAI} from "@langchain/openai";

const modelsCache = {};

const getLLMClient = async (model) => {
  if (modelsCache?.[model]) {
    return modelsCache[model];
  }
  try {
    const openAIApiKey = process.env.OPENAI_API_KEY;
    if (!openAIApiKey) {
      throw new Error("OpenAI API key not set in environment variables");
    }
    const llm = new ChatOpenAI({
      model: model,
      apiKey: openAIApiKey,
      maxTokens: 4096,
    });
    modelsCache[model] = llm;
    return llm;
  } catch (error) {
    throw new Error("Error creating OpenAI client");
  }
};

const getLLMCompletion = async (messages, model="gpt-4o") => {
  try {
    const llm = await getLLMClient(model);
    const response = await llm.invoke(messages);
    return response.content;
  } catch (error) {
    throw new Error("Error getting LLM completion");
  }
};

export {getLLMClient, getLLMCompletion};