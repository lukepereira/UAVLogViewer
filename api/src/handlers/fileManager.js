export class FileManager {
  static async upload(req, res) {
    console.debug('Upload request received');
    if (!req?.files?.length) {
      console.debug('No files uploaded');
      return res.status(400).json({ error: 'No file uploaded' });
    }

    // Files are stored inside a form using Deep Chat request FormData format:
    // https://deepchat.dev/docs/connect
    if (req.files) {
      console.log('Files:');
      console.log(req.files);
    }
    // When sending text along with files, it is stored inside the request body using the Deep Chat JSON format:
    // https://deepchat.dev/docs/connect
    if (Object.keys(req.body).length > 0) {
      console.log('Text messages:');
      // message objects are stored as strings and they will need to be parsed (JSON.parse) before processing
      console.log(req.body);
    }
    // Sends response back to Deep Chat using the Response format:
    // https://deepchat.dev/docs/connect/#Response
    res.status(200).json({ status: 'Files received successfully' });
  }
}
