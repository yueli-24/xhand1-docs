import { COOKIE_NAME } from "@shared/const";
import { getSessionCookieOptions } from "./_core/cookies";
import { systemRouter } from "./_core/systemRouter";
import { publicProcedure, router } from "./_core/trpc";
import { z } from "zod";
import { admins, docContents } from "../drizzle/schema";
import { getDb } from "./db";
import { eq } from "drizzle-orm";
import * as crypto from "crypto";

// Simple password hashing
function hashPassword(password: string): string {
  return crypto.createHash("sha256").update(password).digest("hex");
}

// Admin session management (simple in-memory for now)
const adminSessions = new Map<string, { adminId: number; username: string; expiresAt: number }>();

function generateSessionToken(): string {
  return crypto.randomBytes(32).toString("hex");
}

function setAdminSession(res: any, adminId: number, username: string) {
  const token = generateSessionToken();
  const expiresAt = Date.now() + 24 * 60 * 60 * 1000; // 24 hours
  adminSessions.set(token, { adminId, username, expiresAt });
  res.cookie("admin_session", token, {
    httpOnly: true,
    secure: process.env.NODE_ENV === "production",
    sameSite: "lax",
    maxAge: 24 * 60 * 60 * 1000,
  });
  return token;
}

function getAdminSession(req: any): { adminId: number; username: string } | null {
  const token = req.cookies?.admin_session;
  if (!token) return null;
  
  const session = adminSessions.get(token);
  if (!session) return null;
  
  if (Date.now() > session.expiresAt) {
    adminSessions.delete(token);
    return null;
  }
  
  return { adminId: session.adminId, username: session.username };
}

function clearAdminSession(req: any, res: any) {
  const token = req.cookies?.admin_session;
  if (token) {
    adminSessions.delete(token);
  }
  res.clearCookie("admin_session");
}

export const appRouter = router({
    // if you need to use socket.io, read and register route in server/_core/index.ts, all api should start with '/api/' so that the gateway can route correctly
  system: systemRouter,
  auth: router({
    me: publicProcedure.query(opts => opts.ctx.user),
    logout: publicProcedure.mutation(({ ctx }) => {
      const cookieOptions = getSessionCookieOptions(ctx.req);
      ctx.res.clearCookie(COOKIE_NAME, { ...cookieOptions, maxAge: -1 });
      return {
        success: true,
      } as const;
    }),
  }),

  // Admin routes for document management
  admin: router({
    // Admin login
    login: publicProcedure
      .input(z.object({
        username: z.string(),
        password: z.string(),
      }))
      .mutation(async ({ input, ctx }) => {
        const db = await getDb();
        if (!db) throw new Error("Database not available");
        
        const admin = await db.select().from(admins).where(eq(admins.username, input.username)).limit(1);
        
        if (admin.length === 0) {
          throw new Error("用户名或密码错误");
        }
        
        const passwordHash = hashPassword(input.password);
        if (admin[0].passwordHash !== passwordHash) {
          throw new Error("用户名或密码错误");
        }
        
        const token = setAdminSession(ctx.res, admin[0].id, admin[0].username);
        console.log('[Admin Login] Session created:', { adminId: admin[0].id, username: admin[0].username, token });
        
        return { success: true, adminId: admin[0].id, username: admin[0].username };
      }),
    
    // Check admin session - use OAuth user
    me: publicProcedure.query(({ ctx }) => {
      // Check if user is logged in via OAuth and has admin role
      if (ctx.user && ctx.user.role === 'admin') {
        return {
          id: ctx.user.id,
          username: ctx.user.name || ctx.user.email,
        };
      }
      return null;
    }),
    
    // Admin logout
    logout: publicProcedure.mutation(({ ctx }) => {
      clearAdminSession(ctx.req, ctx.res);
      return { success: true };
    }),
    
    // Get all documents
    listDocs: publicProcedure.query(async ({ ctx }) => {
      // Check OAuth user role
      if (!ctx.user || ctx.user.role !== 'admin') throw new Error("未授权");
      
      const db = await getDb();
      if (!db) throw new Error("Database not available");
      
      const docs = await db.select().from(docContents);
      return docs;
    }),
    
    // Get document by ID
    getDoc: publicProcedure
      .input(z.object({ docId: z.string() }))
      .query(async ({ input, ctx }) => {
        // Check OAuth user role
        if (!ctx.user || ctx.user.role !== 'admin') throw new Error("未授权");
        
        const db = await getDb();
        if (!db) throw new Error("Database not available");
        
        const doc = await db.select().from(docContents).where(eq(docContents.docId, input.docId)).limit(1);
        return doc.length > 0 ? doc[0] : null;
      }),
    
    // Update document content
    updateDoc: publicProcedure
      .input(z.object({
        docId: z.string(),
        content: z.string(),
      }))
      .mutation(async ({ input, ctx }) => {
        // Check OAuth user role
        if (!ctx.user || ctx.user.role !== 'admin') throw new Error("未授权");
        
        const db = await getDb();
        if (!db) throw new Error("Database not available");
        
        // Check if document exists
        const existing = await db.select().from(docContents).where(eq(docContents.docId, input.docId)).limit(1);
        
        if (existing.length > 0) {
          // Update existing
          await db.update(docContents)
            .set({
              content: input.content,
              updatedBy: ctx.user.id,
            })
            .where(eq(docContents.docId, input.docId));
        } else {
          // Insert new
          await db.insert(docContents).values({
            docId: input.docId,
            content: input.content,
            updatedBy: ctx.user.id,
          });
        }
        
        return { success: true };
      }),
  }),
});

export type AppRouter = typeof appRouter;
