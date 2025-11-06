# API

## Getting started

- After starting the UI server, open a separate terminal
  - Change directory to the API folder: `cd api`
  - Init .env file: `cp .env.example .env`
  - Add your OAI token to the .env file
  - Start the API: `npm start`
- To open the chatbot widget
  - Visit the web app at http://localhost:8080
  - Load flight log data
  - Click (...) to show widgets
  - Click the 'AI assistant' widget

## Chat agent walk-through

### UI

An 'AI Assistant' button icon component was added to the widgets section. A vue-based AI chat library handles rendering the chat interface ([deep-chat](https://deepchat.dev)).

### Log processing

Processing large log files before use in the chat workflow allows us to provide LLM agents with the minimal [sufficient context](https://research.google/pubs/sufficient-context-a-new-lens-on-retrieval-augmented-generation-systems/) to generate better responses.
The processing step enables us to support both [Telemetry log files](https://ardupilot.org/planner/docs/mission-planner-telemetry-logs.html) (`*.tlog`) as well as Dataflash files (`*.bin`) for analysis in the chat.

Initial setup on the UI:

- After a user uploads a flight log and `extractFlightData` is run, additional steps were added in the UI to [construct a logData object](https://github.com/lukepereira/UAVLogViewer/blob/master/src/components/Home.vue#L217) with relevant fields, [generate a stable sessionId](https://github.com/lukepereira/UAVLogViewer/blob/master/src/components/Home.vue#L222C23-L222C48) and [POST the logData to the API](https://github.com/lukepereira/UAVLogViewer/blob/master/src/components/Home.vue#L223)
- The `logData` consists of the logType, the simplified trajectories object constructed by the UI, and the raw messages data from original file
- The `sessionId` is a SHA256 hash of the log file and is stored in the global app state
- The `logData` converted to a JSON file blob uses the session id as a filename and POST'ed to the server as multipart/form-data, which allows handling arbitrarily large files.

Main processing on the API:

- The uploaded log data is stored in disk storage for use by the chat workflow and a success response is returned
- An offline/async [processing step](https://github.com/lukepereira/UAVLogViewer/blob/master/api/src/handlers/logManager.js#L150) is triggered and runs in the background
- A description string is looked up in a pre-computed [documentation object](https://github.com/lukepereira/UAVLogViewer/blob/master/api/src/handlers/agent/prompts/logDocumentation.js#L271) (parsed offline from the wiki with [a script](https://github.com/lukepereira/UAVLogViewer/blob/master/api/src/handlers/agent/prompts/logDocumentation.js))
- A stats object is computed using [simple-statistics](https://simple-statistics.github.io/). Descriptional statistics include count, mean, std, min, median, max, and variance. When associated time data is available, time-series statistics included are slope (regression.m), intercept (regression.b) and r2.
- Raw data is serialized then truncated if it exceeds a threshold or is summarized if it consists of a single repeating value
- The resulting data object is organized into [groupings of similar fields](https://github.com/lukepereira/UAVLogViewer/blob/master/api/src/handlers/agent/prompts/logDocumentation.js#L1) and is saved to disk for use in the chat workflow

Example of pre-processed stats and context

- Dataflash (`*.bin`) stats and context snippet
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
          "rawData": <ommitted>
        }
      ]
    }
  }
  ```
- Telemetry (`*.tlog`) stats and context snippet
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
          "rawData": <omitted>
        }
      ]
    }
  }
  ```

### Agent workflow

The chat workflow uses [langgraph](https://www.langchain.com/langgraph) to manage agent orchestration.

The top-level graph in the chat workflow follows an [orchestrator-worker pattern](https://docs.langchain.com/oss/python/langgraph/workflows-agents#orchestrator-worker) with additional steps to synthesize and add a follow up question.

![Workflow: simple diagram](./agent_workflow.png 'Workflow: simple diagram')

The top-level orchestrator returns a [structured output](https://docs.langchain.com/oss/javascript/langchain/structured-output#structured-output) following a schema contains a relevancy score between 0-1 to conditionally determine which nodes to invoke:

```
{
    logAnalysis: <float between 0 and 1>,
    chatOrHelp: <float between 0 and 1>
}
```

The nested `logAnalysis` subgraph follows a [routing pattern](https://docs.langchain.com/oss/python/langgraph/workflows-agents#routing) to dynamically process relevant and available subparts of the log data in parallel.

![Workflow: full diagram](./agent_workflow_xray.png 'Workflow: full diagram')

Depending on the user query, the chat workflow identifies relevant subparts in the available log data to analyze. It uses a MapReduce-like workflow to dispatch subagents to generate answers for groups of similar subparts of the log data. Each subagent aims to use the minimal sufficient context composed of relevant log data subparts, their precomputed statistics, and their documentation. Responses from subagents are then synthesized into a final response and returned to the user. The analysis workflow is designed to be simple while also capable of generalizing to answer both narrow queries (e.g. examining only one or two subparts of a log file) as well as high-level queries like anomaly detection (e.g. examining all subparts of the logfile).

The nested `logAnalysis` orchestrator returns a [structured output](https://docs.langchain.com/oss/javascript/langchain/structured-output#structured-output) following a schema that contains a relevancy score between 0-1 to conditionally determine which nodes to invoke (provided they have data available to analyse):

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

Each of these groupings consists of semantically similar data fields found in either tlog or bin files, the configuration can be found [here](https://github.com/lukepereira/UAVLogViewer/blob/master/api/src/handlers/agent/prompts/logDocumentation.js). Groupings were generated with an LLM, more rigorous methods using manual review or embeddings could be used.

This design choice was chosen since it generalizes to handle both Telemetry files and Dataflash files, which can contain over 300 different types of [message data keys](https://ardupilot.org/plane/docs/logmessages.html). By precomputing semantically similar clusters of data types and then dynamically dispatching queries to relevant groups in parallel, the chat workflow can scale to highly complex log files while also quickly generating comprehensive responses that are specific to a user's query.

Follow up question or clarification: After synthesizing a response, a relevant follow-up question to explore the data in more depth or to clarify the user's intent is appended to the output.

Help: Users can also ask questions about the chat functionality or the flight log data schema, which is answered using documentation dynamically added to the context.

Out-of-domain questions: Indirect questions outside of the domain of flight log data analyses or documentation can still be answered, but will have a disclaimer string `[LLM: verify]` appended to the response.

Currently `gpt-4.1-mini` is being used throughout, but tradeoffs can be made for using better, more expensive reasoning models with slower responses if desired.

## Future work

- Feature idea:
  - Analyze if flight data violates flight regulation rules/codes, e.g. flying in restricted areas or exceeding altitude, speed, or distance limits set by aviation authorities
  - Diagnostic chat workflow to pull in relevant content from UAV manuals and troubleshooting guides
  - Add more sophisticated time series statistical analysis
  - Provide specific expected/abnormal values for different data types in prompts to better detect anomalies
- Improvements:
  - Add unit tests and E2E tests
  - Validate log data during processing
  - Fix security issues from adding new packages to repo with outdated dependencies
  - Delete log data after some retention period, consider storing in database
  - Persist chat, include 'clear' button, suggest random queries
