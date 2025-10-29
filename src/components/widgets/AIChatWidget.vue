<template>
    <div
        :id="getDivName()"
        v-bind:style="{
        width: width + 'px',
        height: height + 'px',
        top: top + 'px',
        left: left + 'px'
        }"
    >
        <div id='paneContent'>
            <span style="float: right; margin: 3px; cursor: pointer;" @click="close()"> X </span>
            <div class="section">
                <h6>Chat</h6>
                <div class="chat-container">
                    <deep-chat
                        :demo.prop="true"
                        :textInput.prop="{ placeholder: { text: 'Welcome to the demo!' } }"
                        :history.prop="history"
                        />
                </div>
            </div>
        </div>
    </div>
</template>

<script>
import 'deep-chat'
import { store } from '../Globals.js'
import { baseWidget } from './baseWidget'

export default {
    name: 'AIChatWidget',
    mixins: [baseWidget],
    data () {
        return {
            name: 'AIChatWidget',
            state: store,
            width: 410,
            height: 450,
            left: 50,
            top: 50,
            isLoading: false,
            history: [
                { role: 'ai', text: 'Hi, how can I help you?' }
            ]
        }
    }
}
</script>

<style scoped>
    div.section {
        border: 1px solid #ccc;
        width: 96%;
        padding: 5px;
        margin-bottom: 10px;
        box-sizing: border-box;
    }

    div#paneContent {
        height: 100%;
        overflow: auto;
        -webkit-user-select: text;
        -moz-user-select: text;
        -ms-user-select: text;
        user-select: text;
    }

    div#paneContent ul {
        list-style: none;
        line-height: 22px;
        padding: 16px;
        margin: 0;
    }

    .chat-container {
        width: 100%;
        font-family: sans-serif;
        text-align: center;
        justify-content: center;
        display: grid;
    }

    div #paneAIChatWidget {
        padding: 15px;
        min-width: 220px;
        min-height: 150px;
        position: absolute;
        background: rgba(253, 254, 255, 0.856);
        color: #141924;
        font-size: 11px;
        font-weight: 600;
        text-transform: uppercase;
        z-index: 10000;
        box-shadow: 9px 9px 3px -6px rgba(26, 26, 26, 0.699);
        border-radius: 5px;
        user-select: text;
    }

    div #paneAIChatWidget::before {
        content: '\25e2';
        color: #ffffff;
        background-color: rgb(38, 53, 71);
        position: absolute;
        bottom: -1px;
        right: 0;
        width: 17px;
        height: 21px;
        padding: 2px 3px;
        border-radius: 10px 0px 1px 0px;
        box-sizing: border-box;
        align-items: center;
        justify-content: center;
        font-size: 10px;
        cursor: se-resize;
    }

    div #paneAIChatWidget::after {
        content: '\2725';
        color: #2E3F54;
        position: absolute;
        top: 0;
        left: 0;
        width: 18px;
        height: 17px;
        margin-top: -3px;
        padding: 0px 2px;
        box-sizing: border-box;
        align-items: center;
        justify-content: center;
        font-size: 17px;
        cursor: grab;
    }
</style>
