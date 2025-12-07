/**
 * Context-7: Maximum 7 files per query system
 * Rationale: Cognitive load (Miller's Law), token efficiency, latency control
 */

export const CONTEXT_MAX_FILES = 7;

export interface ContextFile {
  id: string;
  name: string;
  path: string;
  size: number;
  type: 'lesson' | 'chapter' | 'section' | 'document';
  uploadedAt: number;
  source: 'textbook' | 'external';
}

export interface Context7State {
  files: ContextFile[];
  selectedCount: number;
  isValid: boolean;
  error?: string;
}

export interface Context7Action {
  type: 'ADD' | 'REMOVE' | 'CLEAR' | 'VALIDATE';
  file?: ContextFile;
}

export function isContext7Valid(files: ContextFile[]): boolean {
  return files.length > 0 && files.length <= CONTEXT_MAX_FILES;
}

export function getContext7Status(files: ContextFile[]): string {
  const count = files.length;
  return `${count} of ${CONTEXT_MAX_FILES} files selected`;
}
