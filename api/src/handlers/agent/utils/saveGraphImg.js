import * as fsPromises from 'node:fs/promises';

export const saveGraphImage = async (graph, outfile = 'graph_image.png') => {
  console.log(`Saving graph image to ${outfile}`);
  const drawableGraph = await graph.getGraphAsync({ xray: false });
  const drawableGraphXray = await graph.getGraphAsync({ xray: true });

  const image1 = await drawableGraph.drawMermaidPng({
    withStyles: true,
    curveStyle: 'linear',
    backgroundColor: 'white',
  });
  const image2 = await drawableGraphXray.drawMermaidPng({
    withStyles: true,
    curveStyle: 'linear',
    backgroundColor: 'white',
  });

  const imageBuffer = new Uint8Array(await image1.arrayBuffer());
  await fsPromises.writeFile(outfile, imageBuffer);
  const imageBufferXray = new Uint8Array(await image2.arrayBuffer());
  await fsPromises.writeFile(outfile.replace('.png', '_xray.png'), imageBufferXray);
};
