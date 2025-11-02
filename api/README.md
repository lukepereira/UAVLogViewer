# API

## Getting started

- After starting the UI server, open a seperate terminal
  - Change directory to the API folder: `cd api`
  - Init .env file: `cp .env.example .env`
  - Add your OAI token to the .env file
  - Start the API: `npm start`
- To open the chatbot widget
  - Visit the web app at http://localhost:8080
  - Load flight log data
  - Click (...) to show widgets
  - Click the 'AI assistant' widget

## Chat agent overview

- After uploading a UAV flight log or loading the sample:
  - A stable sessionId is generated based on a hash of the log data
  - Log data is uploaded as a multipart/form-data file to the server using the sessionId in the filename. See [log documentation](./api/src/handlers/agent/prompts/logDocumentation.js) for details on the available data fields
  - Both Telemetry files (`*.tlog`) and Dataflash files (`*.bin`) are supported and can be analyzed by the chat
  - After storing log data in disk storage, an async pre-processing step is triggered to collect statistical info and relevant documentation for the log file which is also saved to disk. All available numerical series in the flight log data are analyzed using [simple-statistics](https://simple-statistics.github.io/). Descriptional statistics include count, mean, std, min, median, max, and variance. When associated time data is parsed, time-series statistics included are slope (regression.m), intercept (regression.b) and r2.
- On the UI, after clicking the chat widget, a [deep-chat](https://deepchat.dev) interface is rendered with a connection to the API using the current sessionId
- Chat functionality includes:
  - Flight log analysis: Depending on the user query, the chat workflow identifies relevant subparts in the available log data to analyze. It uses a MapReduce-like workflow to dispatch subagents to generate answers for groups of similar subparts of the log data. Each subagent aims to use the minimal sufficent context composed of relevant log data subparts, their precomputed statistics, and their documentation. Responses from subagents are then synthesized into a final response and returned to the user.
  - Query types: This workflow is designed to be simple while also capable of generalizing to answer both narrow queries (e.g. examining only one or two subparts of a log file) as well as high-level queries like anomaly detection (e.g. examining all subparts of the logfile).
  - Follow up question or clarification: After synthesizing a response, a relevant follow-up question to explore the data in more depth or to clarifiy the user's intent is appended to the output.
  - Help: Users can also ask questions about the chat functionality or the flight log data schema, which is answered using documentation dynamically added to the context.
  - Out-of-domain questions: Indirect questions outside of the domain of flight log data analyses or documentation can still be answered, but will have a disclaimer string `[LLM: verify]` appended to the response.

## Chat agent workflow

The chat workflow uses [langgraph](https://www.langchain.com/langgraph) to manage agent orchestration. Currently `gpt-4.1-mini` is being used throughout, but tradeoffs can be made for using better, more expensive reasoning models with slower responses if desired.

The top-level graph in the chat workflow follows an [orchestrator-worker pattern](https://docs.langchain.com/oss/python/langgraph/workflows-agents#orchestrator-worker) with an additional step to add a follow up question.

The top-level orchestrator returns a [structured output](https://docs.langchain.com/oss/javascript/langchain/structured-output#structured-output) following a schema that containing a relevancy score between 0-1 to conditionally determine which nodes to invoke:

```
{
    logAnalysis: <float between 0 and 1>,
    chatOrHelp: <float between 0 and 1>
}
```

![Workflow: simple diagram](./agent_workflow.png 'Workflow: simple diagram')

The nested `logAnalysis` subgraph follows a [routing pattern](https://docs.langchain.com/oss/python/langgraph/workflows-agents#routing) to dynamically process relevant and available subparts of the log data in parallel.

The nested `logAnalysis` orchestrator returns a [structured output](https://docs.langchain.com/oss/javascript/langchain/structured-output#structured-output) following a schema that containing a relevancy score between 0-1 to conditionally determine which nodes to invoke (provided they have data available to analyse):

```
{
    "inertialMotion": <float between 0 and 1>,
    "gpsNavigation": <float between 0 and 1>,
    "controlAutopilot": <float between 0 and 1>,
    "actuatorsMotors": <float between 0 and 1>,
    "airEnvironment": <float between 0 and 1>,
    "simulationSitl": <float between 0 and 1>,
    "communicationTelemetry": <float between 0 and 1>,
    "loggingReplay": <float between 0 and 1>,
    "powerBattery": <float between 0 and 1>,
    "camerasSensors": <float between 0 and 1>,
    "safetyMisc": <float between 0 and 1>
}
```

![Workflow: full diagram](./agent_workflow_xray.png 'Workflow: full diagram')

(Blurry png seems to be a known issue with the typescript langgraph package)

## Example of pre-processed stats and context

Dataflash (`*.bin`) stats and context snippet

```json
{
  "controlAutopilot": {
    "ATT": [
      {
        "data_key": "DesPitch",
        "description": "vehicle desired pitch (Unit: deg)",
        "stats": {
          "count": 90348,
          "mean": 0.00748118386682605,
          "std": 0.012099196071873703,
          "min": 0,
          "max": 0.42,
          "median": 0.01,
          "q25": 0,
          "q75": 0.01,
          "trend": {
            "slope": 5.091439526635709e-8,
            "intercept": 0.0014686796504425424,
            "r2": 0.07528450437386824
          }
        },
        "rawData": "ommitted"
      }
    ]
  }
}
```

Telemetry (`*.tlog`) stats and context snippet

```json
{
  "gpsNavigation": {
    "GPS_RAW_INT": [
      {
        "data_key": "alt",
        "description": "Altitude (MSL) (Unit: mm)",
        "stats": {
          "count": 799,
          "mean": 611922.0650813517,
          "std": 18753.86315121873,
          "min": 586640,
          "max": 644480,
          "median": 623220,
          "q25": 587850,
          "q75": 626300,
          "trend": {
            "slope": -0.10239560634146494,
            "intercept": 623002.6209506687,
            "r2": 0.10018632499952063
          }
        },
        "rawData": "omitted"
      }
    ]
  }
}
```

## Future work

- Feature idea:
  - Analyze if flight data violates flight regulation rules/codes, e.g. flying in restricted areas or exceeding altitude, speed, or distance limits set by aviation authorities
  - Persist chat, include 'clear' button, suggest random queries
  - Add more sophisticated time series statistical analysis
  - Provide specific expected/abnormal values for different data types in prompts to better detect anomalies
- Improvements:
  - Add unit tests and E2E tests
  - Validate log data during processing
  - Fix security issues from adding new packages to repo with outdated dependencies
  - Delete log data after some retention period, consider storing in database
