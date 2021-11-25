#!/usr/bin/env python
# -*- coding: utf-8 -*-
#用于分支测试
from collections import deque

import sortedcontainers
import heapq


class Queue(object):
    def __init__(self):
        self._items = deque([])

    def push(self, item):
        self._items.append(item)

    def pop(self):
        return self._items.popleft() \
            if not self.empty() else None

    def empty(self):
        return len(self._items) == 0

    def find(self, item):
        return self._items.index(item) if item in self._items else None


class Stack(object):
    def __init__(self):
        self._items = list()

    def push(self, item):
        self._items.append(item)

    def pop(self):
        return self._items.pop() if not self.empty() else None

    def empty(self):
        return len(self) == 0

    def __len__(self):
        return len(self._items)


class PriorityQueue(object):
    def __init__(self, node):
        self._queue = [node]

    def push(self, node):
        self._queue.append(node)
        self._queue.sort(key = lambda x: (str(x.f)+str(x.h)))

    def pop(self):
        self._queue.sort(key=lambda x: (str(x.f)+str(x.h)))
        temp = self._queue[0]
        del self._queue[0]
        return temp

    def empty(self):
        return len(self._queue) == 0

    def compare_and_replace(self, i, node):
            if node.f < self._queue[i].f:
                del self._queue[i]
                self._queue.append(node)
            self._queue.sort(key=lambda x: (str(x.f)+str(x.h)))

    def find(self, node):
        try:
            loc = self._queue.index(node)
            return loc
        except ValueError:
            return None

class PrioQueue(object):
    def __init__(self, node):
        self.a = heapq.heappop(node)

    def push(self, node):
        self._queue.append(node)
        self._queue.sort(key = lambda x: (str(x.f)+str(x.h)))

    def pop(self):
        self._queue.sort(key=lambda x: (str(x.f)+str(x.h)))
        temp = self._queue[0]
        del self._queue[0]
        return temp

    def empty(self):
        return len(self._queue) == 0

    def compare_and_replace(self, i, node):
            if node.f < self._queue[i].f:
                del self._queue[i]
                self._queue.append(node)
            self._queue.sort(key=lambda x: (str(x.f)+str(x.h)))

    def find(self, node):
        try:
            loc = self._queue.index(node)
            return loc
        except ValueError:
            return None



class Set(object):
    def __init__(self):
        self._items = set()

    def add(self, item):
        self._items.add(item)

    def remove(self, item):
        self._items.remove(item)


class Dict(object):
    def __init__(self):
        self._items = dict()

    def add(self, key, value):
        self._items.update({key: value})

    def remove(self, key):
        self._items.pop(key, None)

    def find(self, key):
        return self._items[key] if key in self._items else None
