Примеры работы с модулем и его at командами можно найти на канале Codius
Библиотека пока что готова для работы только с atmega328p и lgt328p (предпочтительнее из за наличия встроенного ЦАП и бОльших частот). Для обычной avr использовал ядро от Гайвера.
Переменные пинов НЕ задаются через конструктор класса чтобы компилятор точно вырезал лишние проверки https://alexgyver.ru/lessons/code-optimisation/ (Вырезание условий и свитчей)
Я использовал расширение visual micro для Visual Studio. Тк среда Arduino компилирует каждый файл по отдельности, а потом линкует их между собой то из заголовочных файлов не видны внешние дефайны и зависимости.
Поэтому для обьявления константных пинов из других файлов я использовал ключевое слово extern. Создать шаблонный класс не получилось, постоянно выходила ошибка компиляции - почему я не разобрался.

Библиотека SoftwareSerial под lgt немного изменена для использования uart с открытым стоком (обычная - push/pull). Чтобы не использовать согласование уровней с sim800(3.1в), использует внутренннюю подтяжку модуля на пине rx (~70кОм) и отключает подтяжку rx на сторону ардуины
