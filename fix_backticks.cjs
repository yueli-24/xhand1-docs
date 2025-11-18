const fs = require('fs');

const filePath = 'client/src/lib/docs.ts';
let content = fs.readFileSync(filePath, 'utf8');

// 在content字段的模板字符串中，将```替换为\`\`\`
// 这个正则会匹配content: `...` 中的所有```
content = content.replace(/```/g, '\\`\\`\\`');

fs.writeFileSync(filePath, content, 'utf8');
console.log('Fixed backticks in docs.ts');
