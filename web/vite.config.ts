import { defineConfig } from "vite";

export default defineConfig({
  // serve the repo-level assets/ dir (field.glb, gamepiece.glb) at /*
  publicDir: "public",
});
