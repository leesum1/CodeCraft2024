#pragma once

struct Ship {
  int id;
  int capacity;
  int status;
  int berch_id;

  Ship() {}
  Ship(int id, int capacity, int status, int berch_id)
      : id(id), capacity(capacity), status(status), berch_id(berch_id) {}
};