import { Annotation } from "@langchain/langgraph";

export const ChatAgentStateAnnotation = Annotation.Root({
  messages: Annotation({
    reducer: (x, y) => x.concat(y),
  }),
});
