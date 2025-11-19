const fs = require('fs');
const path = require('path');

const docsPath = path.join(__dirname, 'client/src/lib/docs.ts');
let content = fs.readFileSync(docsPath, 'utf8');

// 在模板字符串的content字段中，将单个反引号转义
// 但要避免转义已经转义的反引号
content = content.replace(/`([^`\\]+)`/g, (match, p1) => {
  // 只在content字段内部转义
  return `\\`${p1}\\``;
});

fs.writeFileSync(docsPath, content, 'utf8');
console.log('Fixed single backticks in docs.ts');
