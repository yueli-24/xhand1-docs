import { drizzle } from "drizzle-orm/mysql2";
import { admins } from "../drizzle/schema.ts";
import * as crypto from "crypto";

// Simple password hashing using crypto (Node.js built-in)
function hashPassword(password) {
  return crypto.createHash("sha256").update(password).digest("hex");
}

async function initAdmin() {
  const db = drizzle(process.env.DATABASE_URL);
  
  const username = "tsuki";
  const password = "1Q2w3E4r";
  const passwordHash = hashPassword(password);
  
  try {
    await db.insert(admins).values({
      username,
      passwordHash,
    });
    
    console.log("✅ Admin user created successfully!");
    console.log(`Username: ${username}`);
    console.log(`Password: ${password}`);
  } catch (error) {
    if (error.code === "ER_DUP_ENTRY") {
      console.log("ℹ️  Admin user already exists");
    } else {
      console.error("❌ Failed to create admin user:", error);
      throw error;
    }
  }
  
  process.exit(0);
}

initAdmin();
