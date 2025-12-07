export interface SearchQuery {
  text: string;
  filters?: {
    chapterId?: string;
    type?: 'lesson' | 'section' | 'all';
  };
  limit?: number;
}

export interface SearchResult {
  id: string;
  type: 'lesson' | 'section';
  title: string;
  chapterId: string;
  chapterName: string;
  lessonId?: string;
  sectionId?: string;
  preview: string;
  relevanceScore: number;
  url: string;
}

export interface SearchIndex {
  version: string;
  builtAt: number;
  entries: SearchIndexEntry[];
}

export interface SearchIndexEntry {
  id: string;
  type: 'lesson' | 'section';
  title: string;
  content: string;
  chapterId: string;
  lessonId?: string;
  url: string;
  tokens: string[];
}
