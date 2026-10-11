const { chromium } = require('/home/claude/node_modules/playwright');
const input = process.argv[2] || '/home/claude/oss/pdfbuild/instructable.html';
const output = process.argv[3] || '/mnt/user-data/outputs/Pluvio_M5_Build_Guide.pdf';
(async () => {
  const b = await chromium.launch({ executablePath: '/opt/pw-browsers/chromium-1194/chrome-linux/chrome' });
  const p = await b.newPage();
  await p.goto('file://' + input, { waitUntil: 'networkidle' });
  await p.waitForTimeout(1500);
  await p.pdf({ path: output, printBackground: true, format: 'A4', preferCSSPageSize: true });
  await b.close();
  console.log('pdf done ->', output);
})();
