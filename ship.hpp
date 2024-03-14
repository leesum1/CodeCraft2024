#pragma once

class Ship {

public:
  int id;
  int capacity;
  int status;
  int berch_id;

  explicit Ship() {
    this->id = 0;
    this->capacity = 0;
    this->status = 0;
    this->berch_id = 0;
  }
};