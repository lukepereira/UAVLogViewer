import express from 'express';
import cors from 'cors';
import dotenv from 'dotenv';
import multer from 'multer';
import fs from 'fs';
import { Agent } from './handlers/agent/chat.js';
import { LogManager } from './handlers/logManager.js';

dotenv.config();
const app = express();
const port = 3000;

export const corsOptions = {
  origin: function(origin, callback) {
    if (!origin) return callback(null, true);
    const allowedOrigins = ['http://localhost:8080'];
    if (allowedOrigins.includes(origin)) return callback(null, true);
    return callback(new Error(`Not allowed by CORS ${origin}`));
  },
};

const storage = multer.diskStorage({
  destination: (req, file, cb) => {
    const uploadDir = './uploads';
    if (!fs.existsSync(uploadDir)) fs.mkdirSync(uploadDir);
    cb(null, uploadDir);
  },
  filename: (req, file, cb) => {
    cb(null, file.originalname);
  },
});
const upload = multer({ storage });

app.use(cors(corsOptions));
app.use(express.json());
app.options('*', cors(corsOptions));
app.post('/chat', async (req, res, next) => {
  Agent.chat(req.body, res, next);
});
app.post('/upload-log', upload.array('files'), async (req, res) => {
  LogManager.uploadAndProcessLog(req, res);
});

app.listen(port, () => {
  console.log(`Server is running at http://localhost:${port}`);
});
