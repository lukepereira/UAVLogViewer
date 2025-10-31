// Script to scrape docs: https://ardupilot.org/plane/docs/logmessages.html
// Docs are no longer accessible from the wiki github

const sections = document.querySelectorAll(
  'html.writer-html5 body.wy-body-for-nav div.wy-grid-for-nav section.wy-nav-content-wrap div.wy-nav-content div.rst-content div.document div section#onboard-message-log-messages section',
);

const parsedData = Array.from(sections).map(section => {
  // Get section ID
  const sectionId = section.id;

  // Get section title (from h2)
  const title = section.querySelector('h2')?.innerText || '';

  // Get section description (first <p>)
  const description = section.querySelector('p')?.innerText || '';

  // Parse all tables inside the section
  const tables = Array.from(section.querySelectorAll('table')).map(table => {
    const rows = Array.from(table.querySelectorAll('tbody tr')).map(row => {
      const cells = row.querySelectorAll('td p');
      return {
        name: cells[0]?.innerText || '',
        unit: cells[1]?.innerText || '',
        description: cells[2]?.innerText || '',
      };
    });
    return rows;
  });

  return {
    id: sectionId,
    title,
    description,
    tables,
  };
});

console.log(parsedData);
