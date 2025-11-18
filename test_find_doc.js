// 简化的测试
const testPaths = [
  "/docs/about",
  "/docs/quickstart",
  "/docs/sdk",
  "/docs/teleoperation",
  "/docs/applications",
  "/docs/support"
];

console.log("Testing paths that should exist:");
testPaths.forEach(path => {
  console.log(`- ${path}`);
});
